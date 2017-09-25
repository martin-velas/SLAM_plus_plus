/*
								+---------------------------------+
								|                                 |
								|   ***   Directory lookup  ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2006  |
								|                                 |
								|              Dir.h              |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __DIR_INCLUDED
#define __DIR_INCLUDED

/**
 *	@file Dir.h
 *	@author -tHE SWINe-
 *	@date 2006
 *	@brief basic directory lookup and file info functions
 *
 *	@date 2007-07-03
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
 *	@date 2010-01-07
 *
 *	renamed TFileInfo::p_s_Filename() to TFileInfo::p_s_Path(), added TFileInfo::p_s_FileName()
 *	and TFileInfo::p_s_Extension()
 *
 *	added TFileInfo::GetInfo() which updates info about file from the filesystem
 *
 *	added TFileInfo constructor with filename, which enables TFileInfo being used
 *	much like File in java
 *
 *	added TFileInfo::b_exists, which changes behavior of CDirectory::Get_FirstFile()
 *		and CDirectory::Get_NextFile(), which used TFileInfo::b_valid to indicate whether
 *		there are any more files in the directory. access to TFileInfo::b_valid is now limited
 *		only trough TFileInfo::b_Valid() function, so the old incompatible code wouldn't
 *		compile.
 *
 *	added TFileInfo::n_mode
 *
 *	@date 2010-08-10
 *
 *	added code, handling correct construction of TFileInfo for root directories
 *	via constructor / GetInfo() under windows.
 *
 *	@note note creation / last access / last modification times
 *		are not available for root directories under windows
 *
 *	@date 2010-10-28
 *
 *	fixed minor memory leak, caused by not closing FindFirstFile() handles on windows systems.
 *	it didn't manifest as memory leak as memory got allocated on system side, it just made
 *	pagefile grow on frequent use.
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "\#if defined(_WIN32) || defined(_WIN64)".
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2012-07-17
 *
 *	Modified PRIsizeBparams() and PRIsizeBparamsExt() to be able to cope with uint64_t
 *	properly even with old compilers.
 *
 *	Added CPath class for working with file paths, directories and files (untested as of yet).
 *
 *	Fixed a minor bug in linux version of TFileInfo::GetTempFilename() where the
 *	filename contained two redundant 'X' characters (but caused no problems otherwise).
 *
 *	@date 2012-11-04
 *
 *	Compiled under linux, fixed some bugs carried in over time.
 *
 *	@date 2013-11-13
 *
 *	Fixed temp directory lookup, linux temp directory is not always /tmp, e.g. on cluster computers,
 *	temp is often directory, specific to the job id, like "/tmp/pbs.132048.dm2".
 *
 *	@date 2014-03-11
 *
 *	Added std::string versions of CDirectory::CDirectory(), TFileInfo::TFileInfo()
 *	and TFileInfo::SetFilename().
 *
 *	@date 2014-10-15
 *
 *	Added explicit bad path handling in CDirTraversal::Traverse() and CDirTraversal::Traverse2().
 *
 *	@date 2014-12-12
 *
 *	Changed the behavior of CPath::Get_Path() and CPath::Split() to not return the trailing slash,
 *	the rest of the function does not return it either).
 *
 *	Changed behavior of TFileInfo and CDirectory to accept an empty string as a correct input,
 *	it is interpreted as ".", the current working directory.
 *
 */

#include "NewFix.h"

#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX
#include <windows.h>
#include <string.h>
#include <io.h>
#else // _WIN32 || _WIN64
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h> // rmdir()
#include <string.h>
#include <stdlib.h> // getenv()
#endif // _WIN32 || _WIN64
#include <time.h>
#include <string>
#include <algorithm>

#include "Integer.h"
#include "StlUtils.h"

#include "FormatPrefix.h" // PRIsizeB moved here

/**
 *	@brief simple file-info structure
 *
 *	Keeps memory for it's filename
 *
 *	@todo Rewrite this so filename is stored in std::string, get rid of unnecessary
 *		memory management code.
 */
struct TFileInfo {
protected:
	bool b_valid; /**< are data inside this structure valid? */

public:
	/**
	 *	@brief determines wheter contents of this structure are valid
	 *
	 *	@return Returns true in case this structure contains valid
	 *		information about a file, otherwise returns false.
	 *
	 *	@note All file info's should be valid, even if the file doesn't exist
	 *		(b_exists is not set then). In case this function returns false,
	 *		it's most likely program error (not enough memory, ...).
	 */
	inline bool b_Valid() const
	{
		return b_valid;
	}

	bool b_exists; /**< does specified file / directory exist? @note in case file doesn't exist, value of the following fields are undefined */
	bool b_directory; /**< file / directory flag */

	std::string s_filename; /**< file name */

	/**
	 *	@brief access flag names
	 */
	enum {
		flag_Read = 1,							/**< general reading privilege */
		flag_Write = 2,							/**< general writing privilege */
		flag_Execute = 4,						/**< general execute privilege */

		flag_owner_Read = flag_Read,			/**< owner reading privilege */
		flag_owner_Write = flag_Write,			/**< owner writing privilege */
		flag_owner_Execute = flag_Execute,		/**< owner execute privilege */

		flag_group_Read = flag_Read << 3,		/**< group reading privilege */
		flag_group_Write = flag_Write << 3,		/**< group writing privilege */
		flag_group_Execute = flag_Execute << 3,	/**< group execute privilege */

		flag_other_Read = flag_Read << 6,		/**< other reading privilege */
		flag_other_Write = flag_Write << 6,		/**< other writing privilege */
		flag_other_Execute = flag_Execute << 6	/**< other execute privilege */
	};

	int n_mode; /**< combination of flag_[Read|Write] @note this field is only for files, it is undefined for directories */
	int n_flags; /**< access flags (combination of flag_[owner|group|other]_[Read|Write|Execute]) */
	uint32_t n_size_lo; /**< filesize - 32 low bits */
	uint32_t n_size_hi; /**< filesize - 32 high bits */

	/**
	 *	@brief gets file size
	 *
	 *	@return Returns 64-bit file size.
	 */
	inline uint64_t n_Size64() const
	{
		return (uint64_t(n_size_hi) << 32) | n_size_lo;
	}

	/**
	 *	@brief very simple time structure (unification for linux / windows)
	 *
	 *	@note This was derived from windows' SYSTEMTIME structure, see it's
	 *		documentation on MSDN for more details.
	 *
	 *	@todo Supply conversions back to struct FILETIME / struct tm.
	 */
	struct TTime {
		short n_month;	/**< month index (1 - 12) */
		short n_day;	/**< month day index (1 - 31) */
		short n_year;	/**< year (1601 - 30827) */
		short n_hour;	/**< hour (0 - 23) */
		short n_minute;	/**< minute (0 - 59) */
		short n_second;	/**< second (0 - 59) */

		/**
		 *	@brief default constructor
		 *
		 *	Sets all members to -1.
		 */
		inline TTime()
			:n_month(-1), n_day(-1), n_year(-1), n_hour(-1), n_minute(-1), n_second(-1)
		{}

#if defined(_WIN32) || defined(_WIN64)
		/**
		 *	@brief (win32) conversion constructor
		 *
		 *	Takes <tt>struct SYSTEMTIME</tt> as input, always succeeds.
		 *
		 *	@param[in] r_t_time is input time
		 */
		TTime(const SYSTEMTIME &r_t_time);

		/**
		 *	@brief (win32) conversion constructor
		 *
		 *	Takes <tt>struct FILETIME</tt> as input.
		 *
		 *	@param[in] r_t_time is input time
		 *
		 *	@note FILETIME needs to be converted to SYSTEMTIME first, in case conversion
		 *		fails, all members are set to -1.
		 */
		TTime(const FILETIME &r_t_time);
#else // _WIN32 || _WIN64
		/**
		 *	@brief (unix) conversion constructor
		 *
		 *	Takes <tt>struct tm</tt> as input, always succeeds
		 *
		 *	@param[in] p_time is input time
		 */
		TTime(const tm *p_time);
#endif // _WIN32 || _WIN64

		/**
		 *	@brief equality operator
		 *
		 *	@param[in] r_t_time is time to be compared to
		 *
		 *	@return Returns true in case this is equal to r_t_time, otherwise false.
		 */
		bool operator ==(const TTime &r_t_time) const;

		/**
		 *	@brief less-than operator
		 *
		 *	@param[in] r_t_time is time to be compared to
		 *
		 *	@return Returns true in case this is less than r_t_time, otherwise false.
		 */
		bool operator <(const TTime &r_t_time) const;
	};

	/**
	 *	@brief file time type names
	 */
	enum {
		time_Creation = 0,	/**< time of file creation */
		time_LastAccess,	/**< time of last access to a file */
		time_LastWrite		/**< time of last modification */
	};

	TTime p_time[3]; /**< file time (use time_[Creation|LastAccess|LastWrite] as index) */

	/**
	 *	@brief default constructor
	 *
	 *	Doesn't initialize anything but file name, creates invalid file info.
	 *
	 *	@todo Write constructor with file name specification (get file info for specific file,
	 *		eg. for "c:\blah.txt", so TFileInfo could be used without CDirectory)
	 */
	inline TFileInfo()
		:b_valid(false)
	{}

	/**
	 *	@brief default constructor
	 *
	 *	Copies filename and fills the structure with information about the file. In case the file
	 *	doesn't exist, b_valid is set, but b_exists isn't.
	 *
	 *	@param[in] p_s_filename is null-terminated string, containing the file name
	 */
	inline TFileInfo(const char *p_s_filename)
	{
		b_valid = SetFilename(p_s_filename) && GetInfo();
		// set filename and get info about the file
	}

	/**
	 *	@brief default constructor
	 *
	 *	Copies filename and fills the structure with information about the file. In case the file
	 *	doesn't exist, b_valid is set, but b_exists isn't.
	 *
	 *	@param[in] r_s_filename is string, containing the file name
	 */
	inline TFileInfo(const std::string &r_s_filename)
	{
		b_valid = SetFilename(r_s_filename) && GetInfo();
		// set filename and get info about the file
	}

	/**
	 *	@brief copy constructor
	 *
	 *	Copies information about file r_other to this. Note there may not be enough
	 *		memory for file name, check if b_valid is set.
	 *
	 *	@param r_other is file info to be copied
	 */
	inline TFileInfo(const TFileInfo &r_other)
	{
		*this = r_other; // copy values
	}

	/**
	 *	@brief gets file name
	 *	@return Returns filename as null-terminated string
	 *		  or an empty string in case no filename was specified yet.
	 */
	const char *p_s_FileName() const;

	/**
	 *	@brief gets file name
	 *	@param[in] p_s_path is null-terminated string, containing path to the file
	 *	@return Returns filename as null-terminated string.
	 */
	static const char *p_s_FileName(const char *p_s_path);

	/**
	 *	@brief gets file extension
	 *	@return Returns file extension as null-terminated string,
	 *		or an empty string in case there's no extension.
	 */
	const char *p_s_Extension() const;

	/**
	 *	@brief gets file extension
	 *	@param[in] p_s_path is null-trminated string, containing path to the file
	 *	@return Returns file extension as null-terminated string,
	 *		or an empty string in case there's no extension.
	 */
	static const char *p_s_Extension(const char *p_s_path);

	/**
	 *	@brief gets file path
	 *	@return Returns file path (including filename) as null-terminated string,
	 *		  or 0 in case no filename was specified yet.
	 */
	inline const char *p_s_Path() const
	{
		return s_filename.c_str();
	}

	/**
	 *	@brief marks the file as non-existent (but valid)
	 */
	inline void SetNoFile()
	{
		b_valid = true;
		b_exists = false;
	}

	/**
	 *	@brief marks the file information as invalid (to mark error)
	 */
	inline void SetInvalid()
	{
		b_valid = false;
	}

	/**
	 *	@brief sets new filename
	 *
	 *	Sets filename to p_s_new_filename.
	 *
	 *	@param[in] p_s_new_filename is null-terminated string, containing new file name
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note Note p_s_new_filename can be 0 (sets this filename to 0 as well).
	 */
	bool SetFilename(const char *p_s_new_filename);

	/**
	 *	@brief sets new filename
	 *
	 *	Sets filename to r_s_new_filename.
	 *
	 *	@param[in] r_s_new_filename is string, containing new file name
	 *
	 *	@return Returns true on success, false on failure.
	 */
	inline bool SetFilename(const std::string &r_s_new_filename)
	{
		return SetFilename(r_s_new_filename.c_str());
	}

	/**
	 *	@brief copy-operator
	 *
	 *	Copies contents of r_t_file_info to this.
	 *
	 *	@param[in] r_t_file_info is file info to be copied
	 *
	 *	@return Returns true on success, false in case there
	 *		was not enough memory to hold filename string.
	 */
	bool operator =(const TFileInfo &r_t_file_info);

	/**
	 *	@brief generates name for a temporary file
	 *
	 *	Creates name for temporary file in default system temp path.
	 *	If the function succeeds, it creates an empty file, so it
	 *	may be considered thread-safe (no two threads can create the same file).
	 *
	 *	@param[out] r_s_temp_file_name will contain unique temporary file name
	 *	@param[in] p_s_app_id is short application identifier,
	 *		which will be part of temp filename (may not be empty)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note p_s_app_id might get truncated if it's too long (3 characters is usual length)
	 */
	static bool Get_TempFileName(std::string &r_s_temp_file_name, const char *p_s_app_id = "tmp");

	/**
	 *	@brief generates name for a temporary file, in a specified directory
	 *
	 *	Creates name for temporary file in user supplied temp path.
	 *	If the function succeeds, it creates an empty file, so it
	 *	may be considered thread-safe (no two threads can create the same file).
	 *
	 *	@param[out] r_s_temp_file_name will contain unique temporary file name
	 *	@param[in] p_s_directory is path to the directory to put the temp
	 *		file in or null to use the system default temp directory
	 *	@param[in] p_s_app_id is short application identifier,
	 *		which will be part of temp filename (may not be empty)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note p_s_app_id might get truncated if it's too long (3 characters is usual length)
	 */
	static bool Get_TempFileName_Dir(std::string &r_s_temp_file_name,
		const char *p_s_directory, const char *p_s_app_id = "tmp");

#if defined(_WIN32) || defined(_WIN64)
	/**
	 *	@brief updates information about the file
	 *
	 *	Calls OS api to get information about the file (specified by p_s_Path()),
	 *	and fills member variables of the structure.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note Success is even if the file doesn't exist.
	 */
	bool GetInfo();

	/**
	 *	@brief updates information about the file
	 *
	 *	Calls OS api to get information about the file (specified by p_s_Path()),
	 *	and fills member variables of the structure. This overload is optimized for
	 *	getting file information from existing WIN32_FIND_DATA structure (when iterating
	 *	trough files in a directory).
	 *
	 *	@param[in] r_t_find_data contains information about the file (not filename)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note Success is even if the file doesn't exist.
	 */
	bool GetInfo(const WIN32_FIND_DATAA &r_t_find_data);
#else // _WIN32 || _WIN64
	/**
	 *	@brief updates information about the file
	 *
	 *	Calls OS api to get information about the file (specified by p_s_Path()),
	 *	and fills member variables of the structure.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note Success is even if the file doesn't exist.
	 */
	bool GetInfo();
#endif // _WIN32 || _WIN64
};

/**
 *	@brief simple directory class
 *
 *	This class allows for very basic access to directory's files (sub-directories).
 */
class CDirectory {
public:
	/**
	 *	@brief path separator character
	 */
	enum {
#if defined(_WIN32) || defined(_WIN64)
		path_Separator = '\\'	/**< windows path separator character */
#else // _WIN32 || _WIN64
		path_Separator = '/'	/**< unix path separator character */
#endif // _WIN32 || _WIN64
	};

private:
	char *m_p_s_dir_name;

#if defined(_WIN32) || defined(_WIN64)
	HANDLE m_h_prev_file;
#else // _WIN32 || _WIN64
    DIR *m_p_dir;
#endif // _WIN32 || _WIN64

public:
	/**
	 *	@brief default constructor
	 *
	 *	Creates handle to directory with address p_s_dir_name. Call p_s_Name() to find
	 *		out wheter there was enough memory for string copy, otherwise this object
	 *		can't be used to find files (sub-directories).
	 *
	 *	@param[in] p_s_dir_name is directory name (such as "c:\blah" or
	 *		"c:\blah\" or ".\blah" or ".\blah\" or "blah" or "blah\")
	 *
	 *	@note p_s_dir_name is copied and can be freed right after the constructor returns.
	 *
	 *	@todo Rewrite this so name is stored in std::string, get rid of unnecessary
	 *		memory management code.
	 */
	inline CDirectory(const char *p_s_dir_name)
	{
		Initialize(p_s_dir_name);
	}

	/**
	 *	@brief default constructor
	 *
	 *	Creates handle to directory with address p_s_dir_name. Call p_s_Name() to find
	 *		out wheter there was enough memory for string copy, otherwise this object
	 *		can't be used to find files (sub-directories).
	 *
	 *	@param[in] r_s_dir_name is directory name (such as "c:\blah" or
	 *		"c:\blah\" or ".\blah" or ".\blah\" or "blah" or "blah\")
	 *
	 *	@note The contents of r_s_dir_name are copied and
	 *		can be freed right after the constructor returns.
	 *
	 *	@todo Rewrite this so name is stored in std::string, get rid of unnecessary
	 *		memory management code.
	 */
	inline CDirectory(const std::string &r_s_dir_name)
	{
		Initialize(r_s_dir_name.c_str());
	}

	/**
	 *	@brief destructor
	 */
	~CDirectory();

	/**
	 *	@brief gets directory name
	 *
	 *	@return Returns directory name as passed to constructor.
	 *	@return Returns 0 in case there was not enough memory for string copy.
	 */
	inline const char *p_s_Name() const
	{
		return m_p_s_dir_name;
	}

	/**
	 *	@brief finds a file
	 *
	 *	Gets first file in the directory, copies it's data to r_t_file. In case the
	 *		directory doesn't have any files, r_t_file.b_exists is set to false.
	 *
	 *	@param[out] r_t_file on return contains information about found file
	 *
	 *	@return Returns true on success (even if directory is empty - that is marked by
	 *		r_t_file.b_valid), false on failure (possibly inaccesible / non-existant
	 *		directory).
	 *
	 *	@note r_t_file.b_valid is always true, even if the file doesn't exist (aparts
	 *		from when the function fails).
	 */
	bool Get_FirstFile(TFileInfo &r_t_file);

	/**
	 *	@brief finds a file
	 *
	 *	Gets next file in the directory, copies it's data to r_t_file. In case the
	 *		directory doesn't have any more files, r_t_file.b_exists is set to false.
	 *	In case Get_FirstFile() was not called prior calling Get_NextFile, it's called
	 *		automatically.
	 *
	 *	@param[out] r_t_file on return contains information about found file 
	 *
	 *	@return Returns true on success (even if directory does not contain (more)
	 *		files - that is marked by r_t_file.b_valid), false on failure (possibly
	 *		inaccesible / non-existant directory).
	 *
	 *	@note In order to get file list for this folder again, Get_FirstFile()
	 *		must be called (r_t_file contents are only written to, not read from).
	 *	@note r_t_file.b_valid is always true, even if the file doesn't exist (aparts
	 *		from when the function fails).
	 */
	bool Get_NextFile(TFileInfo &r_t_file);

protected:
#if defined(_WIN32) || defined(_WIN64)
	bool GetFileInfo(const WIN32_FIND_DATAA &r_t_find_data, TFileInfo &r_t_file);
#endif // _WIN32 || _WIN64
	void Initialize(const char *p_s_dir_name);

private:
	CDirectory(const CDirectory &r_other); // no-copy, use pointers
	CDirectory &operator =(const CDirectory &r_other); // no-copy, use pointers
};

//#define __CPATH_GET_USER_HOME_DIRECTORY // not in MSVC60!
#define __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT

#if !defined(_WIN32) && !defined(_WIN64)
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h> // get user home directory
#else // !_WIN32 && !_WIN64
#ifdef __CPATH_GET_USER_HOME_DIRECTORY
#include <userenv.h>
#pragma comment(lib, "userenv.lib")
#endif // __CPATH_GET_USER_HOME_DIRECTORY
#ifdef __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT
#include <winnetwk.h>
#pragma comment(lib, "mpr.lib")
#endif // __CPATH_NORMALIZE_NETWORK_DRIVE_SUPPORT
#include <direct.h> // _getcwd
#endif // !_WIN32 && !_WIN64

#if 1

#include "StlUtils.h"
#include <stdio.h>
#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/**
 *	@brief path utilities
 *	@note This is largerly untested. Not to be used until this notice is removed.
 */
class CPath { // todo - add const char * versions, add versions with src = dest
public:
	class CRemoveDirTree {
	protected:
		bool m_b_user_confirm;
		bool m_b_verbose;
		const bool m_b_recycle;
		bool m_b_fails;

	public:
		CRemoveDirTree(bool b_user_confirm, bool b_verbose = false, bool b_recycle = true); // b_recycle is the last to make sure that it doesn't get changed by accident If wanting to change some other param
		bool b_Failed() const;
		bool b_UserConfirm(const char *p_s_path);
		bool operator ()(const TFileInfo &r_t_file);
	};

public:
	enum {
#if defined(_WIN32) || defined(_WIN64)
		case_Sensitive = false, /**< @brief case sensitive filenames flag */
#else // _WIN32 || _WIN64
		case_Sensitive = true, /**< @brief case sensitive filenames flag */
#endif // _WIN32 || _WIN64
		path_Separator = CDirectory::path_Separator /**< @brief path separator character (a slash) */
	};

#if 0
	static bool UnitTests();
#endif // 0

	/**
	 *	@brief gets path separator as a null-terminated string
	 *	@return Returns a null-terminated string containing the path separator.
	 *	@note For the path separator character, use \ref path_Separator.
	 */
	static const char *p_s_Separator()
	{
		static const char p_s_sep[2] = {path_Separator, 0};
		return p_s_sep;
	}

	/**
	 *	@brief compares two strings with respect to filesystem case sensitivity
	 *
	 *	@param[in] p_s_path_a is null-terminated string containing a path or a file name to be compared
	 *	@param[in] p_s_path_b is null-terminated string containing a path or a file name to be compared
	 *
	 *	@return Returns the result of three-way comparison of the given strings,
	 *		honoring the case sensitivity of the filesystem.
	 */
	static inline int n_CompareString(const char *p_s_path_a, const char *p_s_path_b)
	{
		return (case_Sensitive)? strcmp(p_s_path_a, p_s_path_b) : // case sensitive compare
#if defined(_WIN32) || defined(_WIN64)
			_stricmp(p_s_path_a, p_s_path_b); // case insensitive compare
#else // _WIN32 || _WIN64
			strcasecmp(p_s_path_a, p_s_path_b); // case insensitive compare
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@copydoc TFileInfo::Get_TempFileName()
	 */
	static inline bool Get_TempFileName(std::string &r_s_temp_file_name,
		const char *p_s_app_id = "tmp")
	{
		return TFileInfo::Get_TempFileName(r_s_temp_file_name, p_s_app_id);
		// it was here before I started writing CPath. makes more sense here though
	}

	/**
	 *	@copydoc TFileInfo::Get_TempFileName_Dir()
	 */
	static inline bool Get_TempFileName_Dir(std::string &r_s_temp_file_name,
		const char *p_s_directory, const char *p_s_app_id = "tmp")
	{
		return TFileInfo::Get_TempFileName_Dir(r_s_temp_file_name, p_s_directory, p_s_app_id);
		// it was here before I started writing CPath. makes more sense here though
	}

	/**
	 *	@brief gets temporary directory the user can write to
	 *	@param[out] r_s_path is the path to the temporary directory, never ends with a slash
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_TempDirectory(std::string &r_s_path);

	/**
	 *	@brief gets the current working directory
	 *	@param[out] r_s_path is the path to the current working directory, never ends with a slash
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_CurrentDirectory(std::string &r_s_path);

	static bool Get_EnvironmentVariable(std::string &r_s_value, bool &r_b_found, const char *p_s_variable);

	static bool Get_ProgramsDirectory(std::string &r_s_path, bool b_alternative = false);

#ifdef __CPATH_GET_USER_HOME_DIRECTORY

	/**
	 *	@brief gets the current logged-on user home directory
	 *	@param[out] r_s_path is the path to the user home directory, never ends with a slash
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_UserHomeDirectory(std::string &r_s_path);

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
	static bool Normalize(std::string &r_s_path);

	static inline bool Normalize(std::string &r_s_dest, const std::string &r_s_path)
	{
		return stl_ut::Assign(r_s_dest, r_s_path) && Normalize(r_s_dest);
	}

	static inline bool Normalize(std::string &r_s_dest, const char *p_s_path)
	{
		return stl_ut::AssignCStr(r_s_dest, p_s_path) && Normalize(r_s_dest);
	}

	/**
	 *	@brief collapses redundant separators and up-level references
	 *
	 *	A simpler version of Normalize(), intended for easy cases.
	 *	Does not resolve drive letter substitutions, ignores symlinks.
	 *
	 *	@param[in,out] r_s_path is the normalized path to a directory, or a file
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This should *not* be called from inside any ÜberLame general
	 *		purpose function, the caller needs to know that the paths
	 *		will not contain any entities this function might hamper.
	 *	@note The output never ends with slash.
	 */
	static bool WeakNormalize(std::string &r_s_path);

	static inline bool WeakNormalize(std::string &r_s_dest, const std::string &r_s_path)
	{
		return stl_ut::Assign(r_s_dest, r_s_path) && WeakNormalize(r_s_dest);
	}

	static inline bool WeakNormalize(std::string &r_s_dest, const char *p_s_path)
	{
		return stl_ut::AssignCStr(r_s_dest, p_s_path) && WeakNormalize(r_s_dest);
	}

	static bool Join(std::string &r_s_dest, const std::string &r_s_head, const std::string &r_s_tail); // ends with slash only if tail does

	static bool Join(std::string &r_s_dest, const char *p_s_head, const char *p_s_tail); // ends with slash only if tail does

	static inline bool Join(std::string &r_s_head_dest, const std::string &r_s_tail) // ends with slash only if tail does
	{
		return Join(r_s_head_dest, r_s_head_dest, r_s_tail);
	}

	static inline bool Join(std::string &r_s_head_dest, const char *p_s_tail) // ends with slash only if tail does
	{
		return Join(r_s_head_dest, r_s_head_dest.c_str(), p_s_tail);
	}

	static inline bool To_Relative(std::string &r_s_path)
	{
		std::string s_cwd;
		return Get_CurrentDirectory(s_cwd) && To_Relative(r_s_path, s_cwd);
	}

	static bool To_Relative(std::string &r_s_path, const std::string &r_s_current_dir);

	static bool To_Absolute(std::string &r_s_path); // ends with slash only if r_s_path does

	static bool To_Absolute(std::string &r_s_dest, const std::string &r_s_path); // ends with slash only if r_s_path does

	// note that this simply erases the last component of a filename
	// whether the filename actually points to a directory is irrelevant
	// note that loosely speaking, <tt>Get_Path("a/b/c/") + Get_Filename("a/b/c/") = "a/b/c/c"</tt>
	// however, <tt>Get_Path("a/b/c") + Get_Filename("a/b/c") = "a/b/c"</tt>
	static bool Get_Path(std::string &r_s_path, const std::string &r_s_filename); // never ends with slash

	static inline bool Get_Path(std::string &r_s_path) // never ends with slash
	{
		return Get_Path(r_s_path, r_s_path);
	}

	// note that this simply returns only the last component of the path
	// whether it is a directory or a file does not matter
	// note that loosely speaking, <tt>Get_Path("a/b/c/") + Get_Filename("a/b/c/") = "a/b/c/c"</tt>
	// however, <tt>Get_Path("a/b/c") + Get_Filename("a/b/c") = "a/b/c"</tt>
	static bool Get_Filename(std::string &r_s_name, const std::string &r_s_filename);

	static inline bool Get_Filename(std::string &r_s_path)
	{
		return Get_Filename(r_s_path, r_s_path);
	}

	static const char *p_s_Get_Filename(const char *p_s_path);

	/**
	 *	@brief gets extension from a file, pointed to in a filename
	 *
	 *	@param[out] r_s_extension is filled with the extension (including the dot),
	 *		or left empty in case there is no extension
	 *	@param[in] r_s_filename is path to a file, or just a path
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_Extension(std::string &r_s_extension, const std::string &r_s_filename);

	static inline bool Get_Extension(std::string &r_s_path)
	{
		return Get_Extension(r_s_path, r_s_path);
	}

	static bool Split(std::string &r_s_path, std::string &r_s_name, const std::string &r_s_filename); // the path does not end with the slash

	static inline bool Split(std::string &r_s_path_name, std::string &r_s_name)
	{
		return Split(r_s_path_name, r_s_name, r_s_path_name);
	}

	static bool Expand_SystemVariables(std::string &r_s_dest, const std::string &r_s_path);

	static inline bool b_Exists(const std::string &r_s_path)
	{
		TFileInfo t(r_s_path.c_str()); // todo - this is awful waste of performance, getting all those attribs; optimize it
		_ASSERTE(t.b_Valid()); // might not be
		return t.b_exists;
	}

	static inline TFileInfo t_GetInfo(const std::string &r_s_path)
	{
		return TFileInfo(r_s_path.c_str());
	}

	static bool b_Is_Absolute(const std::string &r_s_path);

	/**
	 *	@brief normalize the case of a pathname
	 *
	 *	On Unix-based platforms, this leaves the path unchanged.
	 *	On Windows filesystems, it converts the path to lowercase,
	 *	it also converts forward slashes to backward slashes.
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Normalize_Case(std::string &r_s_dest, const std::string &r_s_path);

	/**
	 *	@brief gets path to the null directory ("/dev/null" on unix sytems)
	 *	@return Returns path to the null directory.
	 */
	static const char *p_s_NullPath();

	static bool Get_NullPath(std::string &r_s_null_path);

	/**
	 *	@brief makes a short filename for verbose, by skipping a prefix
	 *
	 *	@param[in] p_s_filename is file name
	 *	@param[in] n_max_length is maximum output length
	 *
	 *	@return Returns a pair of prefix and filename suffix.
	 */
	static std::pair<const char*, const char*> t_ShortFileName(const char *p_s_filename, size_t n_max_length = 80);

	static bool Make_Directory(const char *p_s_path, bool b_skip_if_exists = false);

	static bool Make_Directories(const char *p_s_path);

	static bool Remove_Directory(const char *p_s_path);

	static bool Recycle(const char *p_s_path); // can be a both file or a dir

	// in windows, this moves to the recycle bin by default. it is considered a bit dangerous to just let it delete everything
	static bool Remove_DirTree(const char *p_s_path, bool b_user_confirm,
		bool b_verbose = false, bool b_recycle = true); // b_recycle is the last to make sure that it doesn't get changed by accident If wanting to change some other param

	static bool Remove_File(const char *p_s_filename);

	static bool Move_File(const char *p_s_new_filename, const char *p_s_old_filename); // may fail to move a file accross disk volumes or other, you may need to copy and delete if this fails

	static bool Copy_File(const char *p_s_new_filename,
		const char *p_s_old_filename, bool b_force_overwrite = false);

	static FILE *p_OpenFile(const char *p_s_filename, const char *p_s_mode);

	static inline FILE *p_OpenFile(const std::string &r_s_filename, const char *p_s_mode)
	{
		return p_OpenFile(r_s_filename.c_str(), p_s_mode);
	}

	static inline FILE *p_OpenFile(const std::string &r_s_filename, std::string &r_s_mode)
	{
		return p_OpenFile(r_s_filename.c_str(), r_s_mode.c_str());
	}

	static bool b_EndsWithSlash(const std::string &r_s_path);

	static bool b_EndsWithSlash(const char *p_s_path);

	//static bool b_Is_Normalized(const std::string &r_s_path); // hard to determine; rename to b_is_sane or something like that
	//static bool b_Is_Normalized(const char *p_s_path); // todo - decide what that should do (find bad characters? find the opposite slashes? find dots and double dots?)
	static bool b_Is_Normalized(const std::string &UNUSED(r_s_path));

	static void Drop_TrailingSlash(std::string &r_s_path);

protected:
#if defined(_WIN32) || defined(_WIN64)
	static bool Canonicalize(std::string &r_s_path);
#endif // _WIN32 || _WIN64
};

#endif // 1

/**
 *	@brief simple recursive directory traversal
 *
 *	Implemented in non-recursive manner (explicit stack), so it wouldn't cause
 *		stack overflow when traversing complex directory trees. To show directory
 *		listing, use:
 *	@code
 *	void OnFile(const TFileInfo &r_t_file)
 *	{
 *		printf((r_t_file.b_directory)? "[%s]\n" : "%s\n", r_t_file.p_s_Path());
 *		// directories printed in brackets
 *	}
 *
 *	int main(int n_arg_num, const char **p_arg_list)
 *	{
 *		_ASSERTE(n_arg_num == 2);
 *		return !CDirTraversal::Traverse(p_arg_list[1], OnFile);
 *		// traverse directory from commandline
 *	}
 *	@endcode
 */
class CDirTraversal {
public:
	/**
	 *	@brief traverses directory
	 *
	 *	Traverses directory p_s_dir_name, all files found are passed to r_file_listener.
	 *		If b_recurse_subdirectories is set, subdirectories are traversed as well.
	 *		All files and subdirectories in a directory are passed to r_file_listener before
	 *		traversing into any of subdirectories (if b_recurse_subdirectories is set,
	 *		that is).
	 *
	 *	@param[in] p_s_dir_name is name of directory to be traversed
	 *	@param[in] r_file_listener is CFileListener object; CFileListener is file listener
	 *		functor, must implement void operator () with a single <tt>const TFileInfo &</tt>
	 *		parameter.
	 *	@param[in] b_recurse_subdirectories is directory recursion flag
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This doesn't give file listener option to early terminate the traversal.
	 *		That is implemented in Traverse2().
	 *	@note Files, passed to file listener are always valid (TFileInfo::b_valid is true).
	 */
	template <class CFileListener>
	static bool Traverse(const char *p_s_dir_name,
		CFileListener &r_file_listener, bool b_recurse_subdirectories = true) // sadly the body must be here otherwise msvc60 fails to specialize
	{
		if(CPath::b_EndsWithSlash(p_s_dir_name)) {
			std::string s_slashless;
			if(!stl_ut::AssignCStr(s_slashless, p_s_dir_name))
				return false;
			CPath::Drop_TrailingSlash(s_slashless);
			TFileInfo info(s_slashless);
			if(!info.b_Valid() || !info.b_exists || !info.b_directory)
				return false;
		} else {
			TFileInfo info(p_s_dir_name);
			if(!info.b_Valid() || !info.b_exists || !info.b_directory)
				return false;
		}
		// explicitly handle bad paths

		CDirectory *p_dir_info;
		if(!(p_dir_info = new(std::nothrow) CDirectory(p_s_dir_name)))
			return false;
		// prepare the first directory handle

		std::vector<CDirectory*> dir_queue_; // don't use this directly
		stl_ut::CPtrContainer_Guard<std::vector<CDirectory*> > dir_queue(dir_queue_); // will delete the pointers in the vector upon deletion

		stl_ut::Reserve_N(*dir_queue, 16);
		if(!dir_queue->capacity()) {
			delete p_dir_info; // !!
			return false;
		}
		dir_queue->push_back(p_dir_info);
		// put it into the queue

		while(!dir_queue->empty()) {
			stl_ut::CUniquePtr<CDirectory> p_dir(dir_queue->back());
			dir_queue->erase(dir_queue->end() - 1);
			// fetch a single record from top of the queue

			size_t n_pos = dir_queue->size();

			TFileInfo t_file_info;
			if(!p_dir->Get_FirstFile(t_file_info)) {
				//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
				//delete p_dir; // handled
				return false; // t_odo - free the pair data
			}
			while(/*t_file_info.b_valid &&*/ t_file_info.b_exists) {
				_ASSERTE(t_file_info.b_Valid());
				r_file_listener((const TFileInfo&)t_file_info);
				// pass file data to listener
			
				if(b_recurse_subdirectories && t_file_info.b_directory) {
					CDirectory *p_subdir_info;
					if(!stl_ut::Reserve_1More(*dir_queue) ||
					   !(p_subdir_info = new(std::nothrow) CDirectory(t_file_info.p_s_Path()))) {
						//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
						//delete p_dir; // handled
						return false;
					}
					// make sure there's enough space in the queue,
					// prepare the subdirectory handle

					dir_queue->insert(dir_queue->begin() + n_pos, p_subdir_info);
				}
				// add the directory to the list to recurse

				if(!p_dir->Get_NextFile(t_file_info)) {
					//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
					//delete p_dir; // handled
					return false;
				}
				// get next file info
			}
			//delete p_dir; // handled
		}

		return true;
	}

	/**
	 *	@brief traverses directory
	 *
	 *	Traverses directory p_s_dir_name, all files found are passed to r_file_listener.
	 *		If b_recurse_subdirectories is set, subdirectories are traversed as well.
	 *		All files and subdirectories in a directory are passed to r_file_listener before
	 *		traversing into any of subdirectories (if b_recurse_subdirectories is set,
	 *		that is).
	 *
	 *	@param[in] p_s_dir_name is name of directory to be traversed
	 *	@param[in] r_file_listener is CFileListener object; CFileListener is file listener
	 *		functor, must implement bool operator () with a single <tt>const TFileInfo &</tt>
	 *		parameter. In case it returns true, traversal continues, in case it returns false,
	 *		Traverse2() returns false immediately.
	 *	@param[in] b_recurse_subdirectories is directory recursion flag
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note Files, passed to file listener are always valid (TFileInfo::b_valid is true).
	 */
	template <class CFileListener>
	static bool Traverse2(const char *p_s_dir_name,
		CFileListener &r_file_listener, bool b_recurse_subdirectories = true) // sadly the body must be here otherwise msvc60 fails to specialize
	{
		if(CPath::b_EndsWithSlash(p_s_dir_name)) {
			std::string s_slashless;
			if(!stl_ut::AssignCStr(s_slashless, p_s_dir_name))
				return false;
			CPath::Drop_TrailingSlash(s_slashless);
			TFileInfo info(s_slashless);
			if(!info.b_Valid() || !info.b_exists || !info.b_directory)
				return false;
		} else {
			TFileInfo info(p_s_dir_name);
			if(!info.b_Valid() || !info.b_exists || !info.b_directory)
				return false;
		}
		// explicitly handle bad paths

		CDirectory *p_dir_info;
		if(!(p_dir_info = new(std::nothrow) CDirectory(p_s_dir_name)))
			return false;
		// prepare the first directory handle

		std::vector<CDirectory*> dir_queue_; // don't use this directly
		stl_ut::CPtrContainer_Guard<std::vector<CDirectory*> > dir_queue(dir_queue_); // will delete the pointers in the vector upon deletion

		stl_ut::Reserve_N(*dir_queue, 16);
		if(!dir_queue->capacity())
			return false;
		dir_queue->push_back(p_dir_info);
		// put it into the queue

		while(!dir_queue->empty()) {
			stl_ut::CUniquePtr<CDirectory> p_dir(dir_queue->back());
			dir_queue->erase(dir_queue->end() - 1);
			// fetch a single record from top of the queue

			size_t n_pos = dir_queue->size();

			TFileInfo t_file_info;
			if(!p_dir->Get_FirstFile(t_file_info)) {
				//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
				//delete p_dir;
				return false; // t_odo - free the pair data
			}
			while(/*t_file_info.b_valid &&*/ t_file_info.b_exists) {
				_ASSERTE(t_file_info.b_Valid());
				if(!r_file_listener((const TFileInfo&)t_file_info)) {
					//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
					//delete p_dir;
					return false;
				}
				// pass file data to listener
			
				if(b_recurse_subdirectories && t_file_info.b_directory) {
					CDirectory *p_subdir_info;
					if(!stl_ut::Reserve_1More(*dir_queue) ||
					   !(p_subdir_info = new(std::nothrow) CDirectory(t_file_info.p_s_Path()))) {
						//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
						//delete p_dir;
						return false;
					}
					// make sure there's enough space in the queue,
					// prepare the subdirectory handle

					dir_queue->insert(dir_queue->begin() + n_pos, p_subdir_info);
				}
				// add the directory to the list to recurse

				if(!p_dir->Get_NextFile(t_file_info)) {
					//std::for_each(dir_queue->begin(), dir_queue->end(), DeleteDir);
					//delete p_dir;
					return false;
				}
				// get next file info
			}
			//delete p_dir;
		}

		return true;
	}

protected:
	//static inline void DeleteDir(CDirectory *p_dir);
};

#if 0

template <class CFileListener>
bool CDirTraversal::Traverse(const char *p_s_dir_name,
	CFileListener &r_file_listener, bool b_recurse_subdirectories /*= true*/)
{
	if(CPath::b_EndsWithSlash(p_s_dir_name)) {
		std::string s_slashless;
		if(!stl_ut::AssignCStr(s_slashless, p_s_dir_name))
			return false;
		CPath::Drop_TrailingSlash(s_slashless);
		TFileInfo info(s_slashless);
		if(!info.b_Valid() || !info.b_exists || !info.b_directory)
			return false;
	} else {
		TFileInfo info(p_s_dir_name);
		if(!info.b_Valid() || !info.b_exists || !info.b_directory)
			return false;
	}
	// explicitly handle bad paths

	CDirectory *p_dir_info;
	if(!(p_dir_info = new(std::nothrow) CDirectory(p_s_dir_name)))
		return false;
	// prepare the first directory handle

	std::vector<CDirectory*> dir_queue;
	stl_ut::Reserve_N(dir_queue, 16);
	if(!dir_queue.capacity())
		return false;
	dir_queue.push_back(p_dir_info);
	// put it into the queue

	while(!dir_queue.empty()) {
		CDirectory *p_dir = dir_queue.back();
		dir_queue.erase(dir_queue.end() - 1);
		// fetch a single record from top of the queue

		size_t n_pos = dir_queue.size();

		TFileInfo t_file_info;
		if(!p_dir->Get_FirstFile(t_file_info)) {
			std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
			delete p_dir;
			return false; // t_odo - free the pair data
		}
		while(/*t_file_info.b_valid &&*/ t_file_info.b_exists) {
			_ASSERTE(t_file_info.b_Valid());
			r_file_listener((const TFileInfo&)t_file_info);
			// pass file data to listener
		
			if(b_recurse_subdirectories && t_file_info.b_directory) {
				CDirectory *p_subdir_info;
				if(!stl_ut::Reserve_1More(dir_queue) ||
				   !(p_subdir_info = new(std::nothrow) CDirectory(t_file_info.p_s_Path()))) {
					std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
					delete p_dir;
					return false;
				}
				// make sure there's enough space in the queue,
				// prepare the subdirectory handle

				dir_queue.insert(dir_queue.begin() + n_pos, p_subdir_info);
			}
			// add the directory to the list to recurse

			if(!p_dir->Get_NextFile(t_file_info)) {
				std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
				delete p_dir;
				return false;
			}
			// get next file info
		}
		delete p_dir;
	}

	return true;
}

template <class CFileListener>
bool CDirTraversal::Traverse2(const char *p_s_dir_name,
	CFileListener &r_file_listener, bool b_recurse_subdirectories /*= true*/)
{
	if(CPath::b_EndsWithSlash(p_s_dir_name)) {
		std::string s_slashless;
		if(!stl_ut::AssignCStr(s_slashless, p_s_dir_name))
			return false;
		CPath::Drop_TrailingSlash(s_slashless);
		TFileInfo info(s_slashless);
		if(!info.b_Valid() || !info.b_exists || !info.b_directory)
			return false;
	} else {
		TFileInfo info(p_s_dir_name);
		if(!info.b_Valid() || !info.b_exists || !info.b_directory)
			return false;
	}
	// explicitly handle bad paths

	CDirectory *p_dir_info;
	if(!(p_dir_info = new(std::nothrow) CDirectory(p_s_dir_name)))
		return false;
	// prepare the first directory handle

	std::vector<CDirectory*> dir_queue;
	stl_ut::Reserve_N(dir_queue, 16);
	if(!dir_queue.capacity())
		return false;
	dir_queue.push_back(p_dir_info);
	// put it into the queue

	while(!dir_queue.empty()) {
		CDirectory *p_dir = dir_queue.back();
		dir_queue.erase(dir_queue.end() - 1);
		// fetch a single record from top of the queue

		size_t n_pos = dir_queue.size();

		TFileInfo t_file_info;
		if(!p_dir->Get_FirstFile(t_file_info)) {
			std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
			delete p_dir;
			return false; // t_odo - free the pair data
		}
		while(/*t_file_info.b_valid &&*/ t_file_info.b_exists) {
			_ASSERTE(t_file_info.b_Valid());
			if(!r_file_listener((const TFileInfo&)t_file_info)) {
				std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
				delete p_dir;
				return false;
			}
			// pass file data to listener
		
			if(b_recurse_subdirectories && t_file_info.b_directory) {
				CDirectory *p_subdir_info;
				if(!stl_ut::Reserve_1More(dir_queue) ||
				   !(p_subdir_info = new(std::nothrow) CDirectory(t_file_info.p_s_Path()))) {
					std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
					delete p_dir;
					return false;
				}
				// make sure there's enough space in the queue,
				// prepare the subdirectory handle

				dir_queue.insert(dir_queue.begin() + n_pos, p_subdir_info);
			}
			// add the directory to the list to recurse

			if(!p_dir->Get_NextFile(t_file_info)) {
				std::for_each(dir_queue.begin(), dir_queue.end(), DeleteDir);
				delete p_dir;
				return false;
			}
			// get next file info
		}
		delete p_dir;
	}

	return true;
}

#endif // 0

/*inline void CDirTraversal::DeleteDir(CDirectory *p_dir)
{
	delete p_dir;
}*/

#endif // !__DIR_INCLUDED
