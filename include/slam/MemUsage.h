/*
								+----------------------------------+
								|                                  |
								|   ***  Memory usage stats  ***   |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2014  |
								|                                  |
								|            MemUsage.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __PROCESS_MEMORY_USAGE_INFO_INCLUDED
#define __PROCESS_MEMORY_USAGE_INFO_INCLUDED

/**
 *	@file include/slam/MemUsage.h
 *	@date 2014
 *	@brief process memory usage statistics
 *	@author -tHE SWINe-
 *
 *	t_odo This is untested on Mac OS.
 */

#include <new>
#include <string>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#if defined(_WIN32) || defined(_WIN64)
#define NOMINMAX // !!
#include <windows.h>
#include <psapi.h>
#if !defined(PSAPI_VERSION) || PSAPI_VERSION == 1
#pragma comment(lib, "psapi.lib") // psapi was moved to kernel32.lib on windows 7 / server 2008 r2
#endif // !PSAPI_VERSION || PSAPI_VERSION == 1
#else // _WIN32 || _WIN64
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#ifdef __APPLE__
#include <mach/mach.h>
#endif // __APPLE__
#endif // _WIN32 || _WIN64
#include "slam/Integer.h"

/**
 *	@brief process memory information class
 */
class CProcessMemInfo {
public:
	/**
	 *	@brief gets process id
	 *	@return Returns the current process pid.
	 */
	static int n_PID()
	{
#if defined(_WIN32) || defined(_WIN64)
		return GetCurrentProcessId();
#else // _WIN32 || _WIN64
		return getpid();
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief gets current memory usage
	 *	@return Returns the current memory usage of the current process (in bytes)
	 *		on success, or -1 on failure (see <tt>orrno</tt> for failure explanation).
	 *	@note On some systems, the granularity might be more than 1 B, specifically
	 *		on linux-based systems it is typically 1 kB.
	 */
	static uint64_t n_MemoryUsage()
	{
#if defined(_WIN32) || defined(_WIN64)
		PROCESS_MEMORY_COUNTERS t_info;
		GetProcessMemoryInfo(GetCurrentProcess(), &t_info, sizeof(t_info));
		return t_info.WorkingSetSize; // in bytes
		// there is also t_info.PeakWorkingSetSize, also in bytes
#elif defined(__APPLE__)
		struct task_basic_info t_info;
		mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
		if(task_info(mach_task_self(), TASK_BASIC_INFO,
		   (task_info_t)&t_info, &t_info_count) != KERN_SUCCESS)
			return uint64_t(-1);
		// resident size is in t_info.resident_size;
		// virtual size is in t_info.virtual_size; // seems thats a relatively constant value, several GB from the start
		return t_info.resident_size;
#else // _WIN32 || _WIN64
#if 0
		struct rusage t_info;
		getrusage(RUSAGE_SELF, &t_info);
		return t_info.ru_idrss; // integral unshared data size, "This field is currently unused on Linux."
#else // 0
		char p_s_temp_file_name[1024];
		{
			const char *p_s_temp;
			{
				p_s_temp = getenv("TMPDIR"); // environment variable
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
			}

			sprintf(p_s_temp_file_name, (p_s_temp[strlen(p_s_temp) - 1] == '/')?
				"%s%sXXXXXX" :  "%s/%sXXXXXX", p_s_temp, "memuse");
			// make a name template

			int n_file;
			if((n_file = mkstemp(p_s_temp_file_name)) < 0)
				return -1; // fail
			close(n_file);
			// create temp file
		}
		// get temp file name

		{
			char p_s_command[1024];
			int n_pid = getpid();
			sprintf(p_s_command, "grep VmData /proc/%d/status > %s", n_pid, p_s_temp_file_name);
			if(system(p_s_command)) {
				remove(p_s_temp_file_name); // don't leave stuff around
				return -1; // fail
			}
			//sprintf(p_s_command, "grep VmData /proc/%d/status > %s", n_pid, "debug.txt");
			//system(p_s_command);
		}
		// format and execute a command that gets the memory usage

		typedef unsigned char uint8;
		double f_value;
		char p_s_unit[64] = {0}; // this might overflow if there is some rubbish in /proc/pid/status
		{
			FILE *p_fr;
			if(!(p_fr = fopen(p_s_temp_file_name, "r")))
				return -1;
			//if(fscanf(p_fr, "VmData: %lf %s ", &f_value, &p_s_unit) < 1) { // dangerous; need to limit unit's size
			if(fscanf(p_fr, "VmData: %lf ", &f_value) < 1) {
				fclose(p_fr);
				remove(p_s_temp_file_name);
				return -1; // fail
			}

			{
				int n_char;
				while(isspace(uint8(n_char = fgetc(p_fr))) && n_char != EOF)
					;
				// skip space

				p_s_unit[0] = n_char; // use this char as well
				int n_pos = (n_char != EOF)? 0 : -1;
				while(n_pos + 1 < 63 && !isspace(uint8(n_char = fgetc(p_fr))) && n_char != EOF)
					p_s_unit[++ n_pos] = n_char;
				assert(n_pos + 1 < 64);
				p_s_unit[n_pos + 1] = 0;
				// read the unit
			}
			// read the unit manually, will not overflow

			fclose(p_fr);
			remove(p_s_temp_file_name);
		}
		// read value and unit from the file

		//printf("debug: %g %s\n", f_value, p_s_unit);

		for(size_t i = 0, n = strlen(p_s_unit); i < n; ++ i) {
			if(isupper(uint8(p_s_unit[i])))
				p_s_unit[i] = tolower(uint8(p_s_unit[i]));
		}
		// convert unit to lowercase

		if(strlen(p_s_unit) > 3)
			return -1; // no idea what that means
		else if(strlen(p_s_unit) == 3) {
			if(p_s_unit[1] != 'i' || p_s_unit[2] != 'b') // "kib", mib", ...
				return -1;
		} else if(strlen(p_s_unit) == 2) {
			if(p_s_unit[1] != 'b') // "kb", "mb", ...
				return -1;
		} else if(*p_s_unit == 0)
			return uint64_t(f_value); // no unit, presumably bytes
		else {
			assert(strlen(p_s_unit) == 1); // otherwise
		}
		// check unit length / pattern

		switch(*p_s_unit) {
		case 'b':
			return uint64_t(f_value); // bytes
		case 'k':
			return uint64_t(f_value * 1024); // kB
		case 'm':
			return uint64_t(f_value * 1048576); // MB
		case 'g':
			return uint64_t((f_value * 1048576) * 1024); // GB
		case 't':
			return uint64_t((f_value * 1048576) * 1048576); // TB
		default:
			return -1; // fail, unknown unit
		};
		// "parse" unit magnitude
#endif // 0
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief gets maximum historic peak memory usage
	 *	@return Returns the peak memory usage of the current process (in bytes)
	 *		on success, or -1 on failure (see <tt>orrno</tt> for failure explanation).
	 *	@note On some systems, the granularity might be more than 1 B, specifically
	 *		on linux-based systems it is typically 1 kB.
	 */
	static uint64_t n_Peak_MemoryUsage()
	{
#if defined(_WIN32) || defined(_WIN64)
		PROCESS_MEMORY_COUNTERS t_info;
		GetProcessMemoryInfo(GetCurrentProcess(), &t_info, sizeof(t_info));
		return t_info.PeakWorkingSetSize; // in bytes
		// there is also t_info.PeakWorkingSetSize, also in bytes
#elif defined(__APPLE__)
		struct rusage t_info;
		getrusage(RUSAGE_SELF, &t_info);
		return t_info.ru_maxrss; // that should be the peak
#else // _WIN32 || _WIN64
#if 0
		struct rusage t_info;
		getrusage(RUSAGE_SELF, &t_info);
		return t_info.ru_maxrss; // integral unshared data size, "This field is currently unused on Linux."
#else // 0
		char p_s_temp_file_name[1024];
		{
			const char *p_s_temp;
			{
				p_s_temp = getenv("TMPDIR"); // environment variable
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
			}

			sprintf(p_s_temp_file_name, (p_s_temp[strlen(p_s_temp) - 1] == '/')?
				"%s%sXXXXXX" :  "%s/%sXXXXXX", p_s_temp, "memuse");
			// make a name template

			int n_file;
			if((n_file = mkstemp(p_s_temp_file_name)) < 0)
				return -1; // fail
			close(n_file);
			// create temp file
		}
		// get temp file name

		{
			char p_s_command[1024];
			int n_pid = getpid();
			sprintf(p_s_command, "grep VmPeak /proc/%d/status > %s", n_pid, p_s_temp_file_name);
			if(system(p_s_command)) {
				remove(p_s_temp_file_name); // don't leave stuff around
				return -1; // fail
			}
			//sprintf(p_s_command, "grep VmData /proc/%d/status > %s", n_pid, "debug.txt");
			//system(p_s_command);
		}
		// format and execute a command that gets the memory usage

		typedef unsigned char uint8;
		double f_value;
		char p_s_unit[64] = {0}; // this might overflow if there is some rubbish in /proc/pid/status
		{
			FILE *p_fr;
			if(!(p_fr = fopen(p_s_temp_file_name, "r")))
				return -1;
			//if(fscanf(p_fr, "VmPeak: %lf %s ", &f_value, &p_s_unit) < 1) { // dangerous; need to limit unit's size
			if(fscanf(p_fr, "VmPeak: %lf ", &f_value) < 1) {
				fclose(p_fr);
				remove(p_s_temp_file_name);
				return -1; // fail
			}

			{
				int n_char;
				while(isspace(uint8(n_char = fgetc(p_fr))) && n_char != EOF)
					;
				// skip space

				p_s_unit[0] = n_char; // use this char as well
				int n_pos = (n_char != EOF)? 0 : -1;
				while(n_pos + 1 < 63 && !isspace(uint8(n_char = fgetc(p_fr))) && n_char != EOF)
					p_s_unit[++ n_pos] = n_char;
				assert(n_pos + 1 < 64);
				p_s_unit[n_pos + 1] = 0;
				// read the unit
			}
			// read the unit manually, will not overflow

			fclose(p_fr);
			remove(p_s_temp_file_name);
		}
		// read value and unit from the file

		//printf("debug: %g %s\n", f_value, p_s_unit);

		for(size_t i = 0, n = strlen(p_s_unit); i < n; ++ i) {
			if(isupper(uint8(p_s_unit[i])))
				p_s_unit[i] = tolower(uint8(p_s_unit[i]));
		}
		// convert unit to lowercase

		if(strlen(p_s_unit) > 3)
			return -1; // no idea what that means
		else if(strlen(p_s_unit) == 3) {
			if(p_s_unit[1] != 'i' || p_s_unit[2] != 'b') // "kib", mib", ...
				return -1;
		} else if(strlen(p_s_unit) == 2) {
			if(p_s_unit[1] != 'b') // "kb", "mb", ...
				return -1;
		} else if(*p_s_unit == 0)
			return uint64_t(f_value); // no unit, presumably bytes
		else {
			assert(strlen(p_s_unit) == 1); // otherwise
		}
		// check unit length / pattern

		switch(*p_s_unit) {
		case 'b':
			return uint64_t(f_value); // bytes
		case 'k':
			return uint64_t(f_value * 1024); // kB
		case 'm':
			return uint64_t(f_value * 1048576); // MB
		case 'g':
			return uint64_t((f_value * 1048576) * 1024); // GB
		case 't':
			return uint64_t((f_value * 1048576) * 1048576); // TB
		default:
			return -1; // fail, unknown unit
		};
		// "parse" unit magnitude
#endif // 0
#endif // _WIN32 || _WIN64
	}
};

#ifndef PRIsizeB

/**
 *	@def PRIsizeB
 *
 *	@brief simple macro for printing file sizes in user-readable format
 *
 *	This macro is used in conjunction with PRIsizeBparams.
 */
#define PRIsizeB "%.*f %s"

/**
 *	@def PRIsizeBparams
 *
 *	@brief simple macro for expanding size in bytes to number of fractional
 *		digits, value and unit
 *
 *	This is used with PRIsizeB for printing sizes. Usage is as follows:
 *
 *	@code
 *	void Test()
 *	{
 *		printf("%d = " PRIsizeB "B\n", 256, PRIsizeBparams(256));
 *		printf("%d = " PRIsizeB "B\n", 1024 + 256, PRIsizeBparams(1024 + 256));
 *		printf("%d = " PRIsizeB "B\n", 1048576, PRIsizeBparams(1048576));
 *	}
 *
 *	output:
 *
 *	256 = 256 B
 *	1280 = 1.25 kB
 *	1048576 = 1.00 MB
 *	@endcode
 *
 *	@param[in] s is size in bytes
 */
#define PRIsizeBparams(s) (((s) < 1024)? 0 : 2), (((s) < 1024)? int(s) : ((s) < 1048576)? \
	double(s) / 1024.0 : ((s) < 1073741824)? double(s) / 1048576.0 : ((s) < 1099511627776)? \
	double(s) / 1073741824.0 : double(s) / 1099511627776.0), (((s) < 1024)? "" : ((s) < 1048576)? "k" : \
	((s) < 1073741824)? "M" : ((s) < 1099511627776)? "G" : "T")

#endif // !PRIsizeB

#endif // !__PROCESS_MEMORY_USAGE_INFO_INCLUDED
