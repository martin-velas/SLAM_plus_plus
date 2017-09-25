/*
								+----------------------------------+
								|                                  |
								|    ***  stdio utilities  ***     |
								|                                  |
								|   Copyright © -tHE SWINe- 2014   |
								|                                  |
								|           StdIOUtils.h           |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __STdIO_UTILS_INCLUDED
#define __STdIO_UTILS_INCLUDED

/**
 *	@file StdIOUtils.h
 *	@date 2014
 *	@author -tHE SWINe-
 *	@brief StdIO utilities
 */

#include "NewFix.h"
#include "CallStack.h"
#include "StlUtils.h"
#include "Integer.h"
#include "Timer.h"
#include "MinMax.h"
#include <stdio.h>

/**
 *	@brief a simple function object that closes stdio file
 */
struct CCloseFile {
	/**
	 *	@brief closes a file
	 *	@param[in] p_pointer is pointer to a file (can be null)
	 */
	inline void operator ()(FILE *p_pointer) const
	{
		if(p_pointer)
			fclose(p_pointer);
	}
};

class CFile_Interface {
protected:
	typedef stl_ut::CUniquePtr<FILE, CCloseFile, CFile_Interface> _TyDerived;

public:
	void Close()
	{
		((_TyDerived*)this)->Destroy();
	}

	bool Open(const char *p_s_filename, const char *p_s_mode)
	{
		Close(); // always, not only on success, would be error prone - would
		// write to a different file on failure if something was already open

		FILE *p_file;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_file, p_s_filename, p_s_mode))
			return false; // fail
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_file = fopen(p_s_filename, p_s_mode)))
			return false; // fail
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400

		*((_TyDerived*)this) = _TyDerived(p_file);
		return true;
	}

	inline operator FILE*() // constructor with FILE* is explicit so assigning one CUniqueFILE_Ptr to another correctly releases ownership, and this makes it possible to call functions such as e.g. fread()
	{
		return ((_TyDerived*)this)->p_Get();
	}

	inline operator const FILE*() const
	{
		return ((_TyDerived*)this)->p_Get();
	}
};

/**
 *	@brief unique pointer specialization for stdio FILE pointers
 */
typedef stl_ut::CUniquePtr<FILE, CCloseFile, CFile_Interface> CUniqueFILE_Ptr;

typedef CUniqueFILE_Ptr CFILE_PtrGuard;

/**
 *	@brief a simple text-based progress indicator
 */
class CTextProgressIndicator {
protected:
	const char *m_p_s_task_name; /**< @brief progress indicator label */
	const double m_f_update_interval; /**< @brief update interval */
	double m_f_next_update; /**< @brief next update time */
	CTimer m_timer; /**< @brief timer */
	std::string m_s_progress; /**< @brief progress string (reuses storage) */
	const int m_n_indent; /**< @brief indentation */
	FILE *m_p_stream; /**< @brief output stream (default stdout) */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] p_s_banner is progress indicator label
	 *	@param[in] f_update_interval is update interval, in seconds (default 0.25)
	 *	@param[in] n_indent is progress indicator indentation, in characters (default 16)
	 *
	 *	@note This does not display the progress indicator; Show() needs to be called.
	 */
	CTextProgressIndicator(const char *p_s_banner, double f_update_interval = .25, int n_indent = 16)
		:m_p_s_task_name(p_s_banner), m_f_update_interval(f_update_interval),
		m_f_next_update(0), m_n_indent(n_indent), m_p_stream(stdout)
	{}

	/**
	 *	@brief sets the output stream (default stream is stdout)
	 *	@param[in] p_stream is pointer to the stream to be used
	 */
	void Set_Stream(FILE *p_stream)
	{
		m_p_stream = p_stream;
	}

	/**
	 *	@brief displays progress indicator with fraction
	 *
	 *	@param[in] n_progress is current progress value
	 *	@param[in] n_goal is goal progress value
	 */
	void Show(size_t n_progress, size_t n_goal)
	{
		_ASSERTE(n_goal > 0); // otherwise causes division by zero

		double f_time = m_timer.f_Time();
		if(f_time < m_f_next_update)
			return;
		m_f_next_update = f_time + m_f_update_interval;
		// do not update too often

		n_progress = min(n_goal, n_progress); // do not exceed goal, otherwise will do out of bounds access
		std::string s_counts, s_progress;
		if(!stl_ut::Format(s_counts, PRIsize " / " PRIsize, n_progress, n_goal) ||
		   !stl_ut::Format(s_progress, "%-20s", "====================" + (20 - (n_progress * 20) / n_goal))) {
			fprintf(m_p_stream, "\r%-*s [" PRIsize " / " PRIsize "]", m_n_indent, m_p_s_task_name, n_progress, n_goal); // fallback
			return;
		}
		_ASSERTE(s_progress.length() >= s_counts.length());
		size_t n_left = (s_progress.length() > s_counts.length())?
			(s_progress.length() - s_counts.length()) / 2 : 0;
		size_t n_right = n_left + s_counts.length();
		_ASSERTE(!s_progress.empty()); // otherwise s_progress.length() - 1 underflows
		for(size_t j = n_left, m = s_progress.length() - 1; j < n_right; ++ j) {
			if(s_counts[j - n_left] != ' ' && (j < m ||
			   s_progress[j] != '=' || s_progress[j + 1] != ' '))
				s_progress[j] = s_counts[j - n_left];
			// keep the leading '=' always visible
		}

		if(!m_s_progress.compare(s_progress))
			return;
		// don't print the same thing again

		fprintf(m_p_stream, "\r%-*s [%s]", m_n_indent, m_p_s_task_name, s_progress.c_str());
		m_s_progress.swap(s_progress);
	}

	/**
	 *	@brief displays bare progress indicator
	 *	@param[in] f_progress is current progress value (in range 0 - 1)
	 */
	void Show(double f_progress)
	{
		double f_time = m_timer.f_Time();
		if(f_time < m_f_next_update)
			return;
		m_f_next_update = f_time + m_f_update_interval;
		// do not update too often

		int n_progress = int(max(0, min(20, 20 - (f_progress * 20))));
		const char *p_s_progress = "====================" + n_progress;
		std::string s_progress;
		if(!stl_ut::Format(s_progress, "%-20s", p_s_progress)) {
			fprintf(m_p_stream, "\r%-*s [%-20s]", m_n_indent, m_p_s_task_name, p_s_progress); // fallback
			return;
		}

		if(!m_s_progress.compare(s_progress))
			return;
		// don't print the same thing again

		fprintf(m_p_stream, "\r%-*s [%s]", m_n_indent, m_p_s_task_name, s_progress.c_str());
		m_s_progress.swap(s_progress);
	}

	/**
	 *	@brief gets time elapsed
	 *	@return Returns time elapsed since the progress indicator object was created, in seconds.
	 */
	double f_Elapsed()
	{
		return m_timer.f_Time();
	}

	/**
	 *	@brief prints a full progress indicator
	 *	@param[in] p_s_additional_info is additional information (printed after
	 *		a colon after the indicator, should not contain newlines, can be null)
	 */
	void Done(const char *p_s_additional_info = 0)
	{
		if(p_s_additional_info) {
			fprintf(m_p_stream, "\r%-*s [%s] : %s\n", m_n_indent, m_p_s_task_name,
				"====================", p_s_additional_info);
		} else
			fprintf(m_p_stream, "\r%-*s [%s]\n", m_n_indent, m_p_s_task_name, "====================");
		m_s_progress.erase();
	}
};

#endif // !__STdIO_UTILS_INCLUDED
