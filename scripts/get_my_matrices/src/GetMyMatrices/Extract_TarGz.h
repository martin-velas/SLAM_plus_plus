#pragma once

#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Dir.h"
#include <vector>
#include <string>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h> // GetCurrentProcessId()
#else // _WIN32 || _WIN64
#include <unistd.h> // getpid()
#endif // _WIN32 || _WIN64

class CUnTarGz {
protected:
	static bool m_b_tar_in_path;
	static bool m_b_bin_tar_in_path;
	static bool m_b_7zip_in_path;
	static bool m_b_pf_7zip_in_path;
	static bool m_b_pfx86_7zip_in_path;

public:
	static bool Extract(const char *p_s_archive_file, const char *p_s_output_dir, const char *p_s_explicit_temp = 0)
	{
		FindPaths();

#if defined(_WIN32) || defined(_WIN64)
		const int n_pid = GetCurrentProcessId();
#else
		const int n_pid = getpid();
#endif
		// avoid collisions in the tarlog.txt

		if(m_b_tar_in_path || m_b_bin_tar_in_path) {
			const char *p_s_tar = (m_b_tar_in_path)? "tar" : "/bin/tar";
			std::string s_cmd;
			/*return CPath::Make_Directories(p_s_output_dir) &&
				stl_ut::Format(s_cmd, "%s tar -xzf \"%s\" -C \"%s\"",
				p_s_tar, p_s_archive_file, p_s_output_dir) && system(s_cmd.c_str());*/
#if defined(_WIN32) || defined(_WIN64)
			std::string s_tar_exe, s_in, s_out;
			if(!stl_ut::AssignCStr(s_in, p_s_archive_file) || !stl_ut::AssignCStr(s_out, p_s_output_dir) ||
			   !CPath::Make_Directories(p_s_output_dir) || !stl_ut::AssignCStr(s_tar_exe, p_s_tar) ||
			   !stl_ut::AppendCStr(s_tar_exe, ".exe"))
				return false;

			std::replace(s_tar_exe.begin(), s_tar_exe.end(), '/', '\\');
			p_s_tar = s_tar_exe.c_str();
			if(*p_s_tar == '\\')
				++ p_s_tar; // skip that one
			// Windows batch won't execute a program with forward slashes

			std::replace(s_in.begin(), s_in.end(), '\\', '/');
			std::replace(s_out.begin(), s_out.end(), '\\', '/');
			p_s_archive_file = s_in.c_str();
			p_s_output_dir = s_out.c_str();
			// GNU tools won't work with backslashes

			return stl_ut::Format(s_cmd, "(set PATH=%s) && %s --gzip --force-local "
				"\"--directory=%s\" -xf \"%s\" 2>> untarlog_%04x.log", (m_b_tar_in_path)? "." : "bin",
				p_s_tar, p_s_output_dir, p_s_archive_file, n_pid) && !system(s_cmd.c_str());
			// need to set path so that tar can execute its gzip
			// need --force-local otherwise GNU tools interpret the ':' in drive letters as the start of a network name

			// tar on windows - worked with cygwin
			//		(needed libintl8-0.19.7-1.tar.xz (under the gettext package),
			//		libiconv2-1.14-3.tar.xz, cygwin-2.5.2-1.tar.xz and tar-1.28-1.tar.xz)
			// did not work with gnu coreutils (fork() not implemented)
			// did not work with the tar from program files/git/bin (failed to allocate stack for child process)
			// note that gzip from gnu coreutils works fine, except that it is x86
#else
			return CPath::Make_Directories(p_s_output_dir) &&
				/*stl_ut::Format(s_cmd, "echo %s --gzip \"--directory=%s\" -xf \"%s\" > tarlog.log",
				p_s_tar, p_s_output_dir, p_s_archive_file) && !system(s_cmd.c_str()) &&*/
				stl_ut::Format(s_cmd, "%s --gzip \"--directory=%s\" -xf \"%s\" 2>> untarlog_%04x.log",
				p_s_tar, p_s_output_dir, p_s_archive_file, n_pid) && !system(s_cmd.c_str());
#endif
		} else if(m_b_7zip_in_path || m_b_pf_7zip_in_path || m_b_pfx86_7zip_in_path) {
			std::string s_7z;
			const char *p_s_7z = "7z.exe";
			if(m_b_pf_7zip_in_path || m_b_pfx86_7zip_in_path) {
				std::string s_program_files;
				if(!CPath::Get_ProgramsDirectory(s_program_files, m_b_pfx86_7zip_in_path) ||
				   !stl_ut::Format(s_7z, (s_program_files.find_first_of(" \t") != std::string::npos)?
				   "\"%s\\7-Zip\\7z.exe\"" : "%s\\7-Zip\\7z.exe", s_program_files.c_str()))
					return false;
				p_s_7z = s_7z.c_str();
			}
			// find 7-zip (could cache this as well but it would make it not thread safe anymore, presumably tiny cost compared to reading files from the disk)

			std::string s_temp_tar;
			if(!CPath::Get_TempFileName_Dir(s_temp_tar, p_s_explicit_temp, "tar"))
				return false;
			// get a temp file name

			std::string s_cmd;
			bool b_result = CPath::Make_Directories(p_s_output_dir) &&
				stl_ut::Format(s_cmd, "(%s e -so \"%s\" 1> \"%s\" ""2>> untarlog_%04x.log"" && " // 7z.exe input file > temp file
				"%s x -y \"-o%s\" \"%s\" "">> untarlog_%04x.log 2>&1"")", // 7z.exe output path temp file 
				p_s_7z, p_s_archive_file, s_temp_tar.c_str(), n_pid,
				p_s_7z, p_s_output_dir, s_temp_tar.c_str(), n_pid) && !system(s_cmd.c_str());
			// sadly, 7z does not implement reading archives from stdin, could avoid the temp
			// file and would just pipe it to the second path using -si
			CPath::Remove_File(s_temp_tar.c_str()); // (try to) remove the temp file regardless of the result
			return b_result;

			// note that 7zip seems to have problems with very large archives (over 8 GB)
		} else {
			fprintf(stderr, "fatal error: found no command-line tool to extract archives with\n");
			return false;
		}
	}

	static void FindPaths()
	{
		if(m_b_tar_in_path || m_b_bin_tar_in_path ||
		   m_b_7zip_in_path || m_b_pf_7zip_in_path ||
		   m_b_pfx86_7zip_in_path)
			return;
		// already resolved

		const char *p_s_null = CPath::p_s_NullPath();

		std::string s_cmd;
		if(TFileInfo("/bin/tar").b_exists)
			m_b_bin_tar_in_path = true;
#if !defined(_WIN32) && !defined(_WIN64)
		else if(stl_ut::Format(s_cmd, "which tar 2>&1 > %s", p_s_null) && !system(s_cmd.c_str())) // use which to see if it is in path
			m_b_tar_in_path = true;
#else // !_WIN32 && !_WIN64
		if(TFileInfo("bin\\tar.exe").b_exists) // does not work with gzip ("bin\tar: Cannot fork: Function not implemented")
			m_b_bin_tar_in_path = true;
#endif // !_WIN32 && !_WIN64
		std::string &s_program_files = s_cmd; // reuse storage
		if(CPath::Get_ProgramsDirectory(s_program_files) &&
		   stl_ut::AppendCStr(s_program_files, "\\7-Zip\\7z.exe") &&
		   TFileInfo(s_program_files).b_exists)
			m_b_pf_7zip_in_path = true;
		else if(CPath::Get_ProgramsDirectory(s_program_files, true) &&
		   stl_ut::AppendCStr(s_program_files, "\\7-Zip\\7z.exe") &&
		   TFileInfo(s_program_files).b_exists)
			m_b_pfx86_7zip_in_path = true;
#if defined(_WIN32) || defined(_WIN64)
		else if(stl_ut::Format(s_cmd, "where.exe 7z.exe > %s 2>&1", p_s_null) && !system(s_cmd.c_str())) // note that where without exe is something else in powershell
			m_b_7zip_in_path = true;
#endif // _WIN32 || _WIN64
	}
};

bool CUnTarGz::m_b_tar_in_path = false;
bool CUnTarGz::m_b_bin_tar_in_path = false;
bool CUnTarGz::m_b_7zip_in_path = false;
bool CUnTarGz::m_b_pf_7zip_in_path = false;
bool CUnTarGz::m_b_pfx86_7zip_in_path = false;
