/*
								+----------------------------------+
								|                                  |
								|   ***   Lexical analyzer   ***   |
								|                                  |
								|   Copyright © -tHE SWINe- 2007   |
								|                                  |
								|            Scanner.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __SCANNER_INCLUDED
#define __SCANNER_INCLUDED

/**
 *	@file Scanner.h
 *	@date 2007
 *	@author -tHE SWINe-
 *	@brief lexical analyzer
 *
 *	@date 2007-09-24
 *
 *	passed code revision
 *
 *	class CScanner made template class with parameter of character type to easily support
 *	simple 7-bit us-ascii parsing with stdlib functions
 *	fixed minor error in CScanner::GrowBuffer() in overflow check
 *	structures TTransition and TState were moved from CScanner to TScannerDrivingTable
 *	renamed CScanner::GetToken_T() to CScanner::GetToken()
 *	CScanner::CTokenEmit was made CScanner template parameter so it's possible
 *	to use functions instead of virtual function objects
 *	TScannerDrivingTable::TState and TScannerDrivingTable::TTransition were made CScanner
 *	template parameters so it's possible to use different versions of driving tables
 *	(like small us-ascii table instead of "default" unicode table)
 *
 *	@date 2008-03-23
 *
 *	fixed memory leak in CScanner (missing destructor which would delete internal buffer)
 *
 *	@date 2008-04-21
 *
 *	modified interface a little bit, now it's easier to write fast token emit functions
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	Added the assumption that the characters will be non-negative.
 *
 */

#include "NewFix.h"

/*
 *	class CTokenEmit
 *		- example implementation of token emit class
 */
/*

class CTokenEmit {
public:
	/+
	 *	bool CTokenEmit::operator ()(const TCharType *p_s_buffer,
	 *		int n_regexp_id, int n_line, int n_column)
	 *		- called by scanner upon reading a token
	 *		- p_s_buffer contains string (TCharType because of unicode) with
	 *		  token literal, n_regexp_id is id of regular expression rule,
	 *		  n_line and n_column are position of token in input data
	 +/
	bool operator ()(const TCharType *p_s_buffer,
		int n_regexp_id, int n_line, int n_column);
};

*/

template <class TCharType, class TStateType, class TTransitionType, class CTokenEmit>
class CScanner {
public:
	/*
	 *	enum CScanner::<unnamed>
	 *		- scanner configuration (currently there's tab size for current column counter only)
	 */
	enum {
		n_Tab_Size = 4
	};

	/*
	 *	class CStringReader
	 *		- example implementation of simple CStreamReader intended
	 *		  to read null-terminated strings
	 */
	/*

	class CStringReader : public CStreamReader {
	protected:
		const char *m_p_s_string;

	public:
		CStringReader(const char *p_s_string)
			:m_p_s_string(p_s_string)
		{}

		virtual bool ReadChar(int &r_n_char)
		{
			if(*m_p_s_string) {
				r_n_char = *m_p_s_string ++;
				return true;
			}
			return false;
		}

		virtual bool UnreadChar(int n_char)
		{
			-- m_p_s_string;
			return true;
		}
	};

	*/

protected:
	const TStateType *m_p_state;
	CTokenEmit m_token_emit;
	int m_n_cur_line, m_n_cur_column;
	TCharType *m_p_buffer, *m_p_buffer_ptr, *m_p_buffer_end;

public:
	/*
	 *	CScanner::CScanner(const TStateType *p_state,
	 *		CTokenEmit t_token_emit, int n_token_emit_num)
	 *		- default constructor
	 *		- creates new instance of CScanner, accepting regular language given by
	 *		  deterministic finite automata specified as set of states, connected by
	 *		  transitions. accepting states have n_regexp_id set to index in CTokenEmit
	 *		  object array (out-of-bounds indices are not accepting)
	 *		- p_state contains finite state machine states
	 *		- p_token_emit contains n_token_emit_num CTokenEmit objects
	 *		- all arrays are only referenced, they have to remain allocated while
	 *		  using scanner
	 */
	CScanner(const TStateType *p_state, CTokenEmit t_token_emit)
		:m_p_state(p_state), m_token_emit(t_token_emit),
		m_n_cur_line(0), m_n_cur_column(0),
		m_p_buffer(0), m_p_buffer_ptr(0), m_p_buffer_end(0)
	{}

	/*
	 *	CScanner::~CScanner()
	 *		- destructor
	 */
	~CScanner()
	{
		if(m_p_buffer)
			delete[] m_p_buffer;
	}

	/*
	 *	template <class CReaderClass>
	 *	bool CScanner::GetToken(CReaderClass &r_input)
	 *		- template function for generating token from input stream
	 *		- r_input is function object, featuring bool ReadChar(int &r_n_char) function
	 *		  which returns true on success and false on EOF, character is put to r_n_char and
	 *		  bool UnreadChar(int n_char) function which is used for returning last read char
	 *		  back to be read next time. this function is called maximally once until calling
	 *		  ReadChar again. return values true success, false failure (reserved for possible
	 *		  i/o errors, etc)
	 *		- returns true on success, false on failure (lexical error, not enough memory)
	 *		- note generated token must be returned by CTokenEmit object called by this function
	 *		  (in case of empty file function returns true but no CTokenEmit is called)
	 *		- use in case aim is maximal speed (but in case you use more CReaderClass types,
	 *		  there will be more instances of this function meaning larger code)
	 */
	template <class CReaderClass>
	bool GetToken(CReaderClass &r_input)
	{
		int n_line = m_n_cur_line;
		int n_column = m_n_cur_column;
		// here lies new token

		m_p_buffer_ptr = m_p_buffer;
		// "clean" input buffer

		const TStateType *p_state = m_p_state;
		do {
			int n_char;
			if(!r_input.ReadChar(n_char)) {
				if(m_p_buffer_ptr == m_p_buffer)
					return true;
				// empty file

				break;
				// EOF in the middle of something
			}
			_ASSERTE(n_char >= 0);
			// get char

			const TStateType *p_next_state = 0;
			for(const TTransitionType *p_transition = p_state->p_transition,
			   *p_end = p_state->p_transition + p_state->n_transition_num;
			   p_transition != p_end; ++ p_transition) {
				if(unsigned(n_char) >= p_transition->n_char_min &&
				   unsigned(n_char) <= p_transition->n_char_max) {
					p_next_state = m_p_state + p_transition->n_state;
					break;
				}
			}
			// find next state

			if(!p_next_state) {
				if(!r_input.UnreadChar(n_char))
					return false;
				// this character is not to be processed

				break;
			}
			p_state = p_next_state;

			if(m_p_buffer_ptr == m_p_buffer_end && !GrowBuffer())
				return false;
			*m_p_buffer_ptr ++ = (TCharType)n_char;
			// accumulate character

			CountChar(n_char);
			// count lines / columns
		} while(p_state->n_transition_num);
		// no point in reading more characters when there are no transitions from current state

		if(p_state->n_regexp_id < 0)
			return false;
		// not accepting state - error (unexpected character / unexpected end of file)

		if(m_p_buffer_ptr == m_p_buffer_end && !GrowBuffer())
			return false;
		*m_p_buffer_ptr ++ = 0;
		// add terminating zero

		return m_token_emit(m_p_buffer, p_state->n_regexp_id, n_line, n_column);
		// emit token
	}

	/*
	 *	inline int CScanner::n_Cur_Line() const
	 *		- returns current line (zero-based index)
	 */
	inline int n_Cur_Line() const
	{
		return m_n_cur_line;
	}

	/*
	 *	inline int CScanner::n_Cur_Column() const
	 *		- returns current column (zero-based index)
	 */
	inline int n_Cur_Column() const
	{
		return m_n_cur_column;
	}

	/*
	 *	void CScanner::CountChar(int n_char)
	 *		- counts lines / columns
	 *		- this is exposed for cases when scanner is not reading all the data
	 *		  but for example some controll tokens only (xml - tags vs. text)
	 */
	void CountChar(int n_char)
	{
		if(n_char == '\n') {
			m_n_cur_column = 0;
			++ m_n_cur_line;
		} else if(n_char != '\t')
			++ m_n_cur_column;
		else {
			m_n_cur_column += n_Tab_Size;
			m_n_cur_column -= m_n_cur_column % n_Tab_Size;
		}
		// count lines / columns
	}

protected:
	bool GrowBuffer()
	{
		size_t n_size = m_p_buffer_end - m_p_buffer;
		size_t n_size2 = 63 | (n_size << 1);
		if(n_size2 <= n_size)
			return false; // overflow
		// determine new size

		TCharType *p_new_buffer;
		if(!(p_new_buffer = new(std::nothrow) TCharType[n_size2]))
			return false;
		// alloc new buffer

		if(m_p_buffer) {
			memcpy(p_new_buffer, m_p_buffer, (m_p_buffer_ptr - m_p_buffer) * sizeof(TCharType));
			delete[] m_p_buffer;
		}
		// copy old data

		m_p_buffer_ptr = p_new_buffer + (m_p_buffer_ptr - m_p_buffer);
		m_p_buffer = p_new_buffer;
		m_p_buffer_end = p_new_buffer + n_size2;
		// update pointers

		return true;
	}
};

#endif // !__SCANNER_INCLUDED
