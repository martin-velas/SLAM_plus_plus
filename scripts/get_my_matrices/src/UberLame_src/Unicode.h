/*
								+----------------------------------+
								|                                  |
								| ***  UTF-8 / UTF-16 support  *** |
								|                                  |
								|   Copyright © -tHE SWINe- 2006   |
								|                                  |
								|            Unicode.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __UNICODE_INCLUDED
#define __UNICODE_INCLUDED

/**
 *	@file Unicode.h
 *	@date 2006
 *	@author -tHE SWINe-
 *	@brief UTF-8 / UTF-16 support
 *
 *	@date 2007-01-10
 *
 *	passed code revision
 *
 *	added CUnicodeFile::SetEncoding() function for proper XML file parsing (late encoding
 *	detection)
 *
 *	CUnicodeFile::n_ForEachChar() was renamed to CUnicodeFile::t_ForEachChar(), value of
 *	functor is no longer converted to int and it doesn't check functor return value any
 *	more; note it doesn't have means to return error code so error flag must be checked
 *	after calling
 *
 *	added CallStack guards, now using it's assert macros
 *
 *	@date 2008-03-04
 *
 *	now using Integer.h header
 *
 *	@date 2008-04-21
 *
 *	added option to disable character processing as it's unnecessary in some cases and
 *	it still have to be called, even if disabled
 *
 *	@date 2008-07-02
 *
 *	fixed CUnicodeFile::n_WriteString() which failed to compile when character processing
 *	was disabled
 *
 *	@date 2009-05-04
 *
 *	fixed mixed windows / linux line endings
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
 */

/*
 *	- if defined, some function for character processing
 *	  (conversion, translation, overflow checks) are compiled
 */
//#define __UNICODE_FILE_ENABLE_CHAR_PROCESSING

#include "CallStack.h"
#include "Integer.h"

#define isunsigned(type) ((type)(-1) > (type)(0))
// macro to determine wheter type is unsigned

#define maxvalueof(type) ((isunsigned(type))? (type)(~0) : (type)((~0U) >> ((sizeof(int) - sizeof(type)) * 8 + 1)))
// macro, used to determine maximal value type can hold (works with integers only)

/*
 *	class CUnicodeFile
 *		- class, encapsulating stdio FILE for fast, buffered innput of classical 'ascii'
 *		  files or unicode (utf-8 or utf-16-le) files with transparent code translation
 *		- text input mode functions have two threshold units, one at custom (user-set) level
 *		  and second at overflow level. both of them can be enabled or disabled, if enabled,
 *		  character can be either discarded, replaced by custom character (separate replacement
 *		  character for each unit
 *		- text output mode functions can read escape sequences and converts them to char values
 *		  which are then written
 *		- binary input mode functions do not apply any wide char translation, result value
 *		  is simply cropped (or padded with MSB zero bits) to fit output data type width
 *		- note unicode can contain such character codes that can have value of control
 *		  characters so it can be necessary to open files in binary mode, even for text i/o
 *		  (depends on your c standard library implementation)
 */
class CUnicodeFile {
protected:
	char *m_p_file_buffer; // up to 16kB file buffer (depends on free memory; writes and / or reads are delayed until necessary which gives better i/o performance)
	int m_n_file_buffer_size;
	char *m_p_file_buffer_cur;
	char *m_p_file_buffer_end; // points to element, next to last valid char in the buffer
#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
	char m_p_string_buffer[11]; // buffer for text-reading mode. in case char translation is enabled, it's possible so a single is read as 10 characters (escape sequence) which are buffered here in case there was not enough space in destination buffer. note this buffer is cleared with every call to Seek() or n_Read() respectively n_Write()
	int m_n_string_buffer_used;
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING
	FILE *m_p_file;
	bool m_b_foreign_file;
	bool m_b_have_bom;
	int m_n_bof_position; // valid position of beginning of file (past BOM)
	int m_n_eof_position; // eof position (valid for reading only - purpose is not to let user read past the end of foreign file section; absolute position in FILE)
	int m_n_encoding;
	bool m_b_writing;
	bool m_b_binary;

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
	struct TThresholdState {
		bool b_enable;
		unsigned int n_threshold;
		int n_mode;
		unsigned int n_replacement_char;
		int n_format_digit_num;
		char p_s_format_string[6]; // "x%0?x" where ? is 2 to 8
	};

	TThresholdState m_t_wide_char_proc;
	TThresholdState m_t_overflow_proc; // threshold is set depending on used data type
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING

	int m_n_error_state;

public:
	enum {
		code_ASCII,
		code_UTF_8,
		code_UTF_16_LE,
		code_Unsupported // big endian and 32-bit UTF-s
	};

	enum {
		seek_Relative = SEEK_CUR,
		seek_Beginning = SEEK_SET,
		seek_End = SEEK_END
	};

	enum {
		wchar_Replace,
		wchar_Translate,
		wchar_Discard,
		wchar_Fail
	};

	enum {
		thresh_User,
		thresh_Overflow
	};

	enum {
		error_NoError,
		error_Unicode,
		error_Threshold,
		error_Overflow,
		error_IO,
		error_NoMemory,
		error_InvalidOperation,
		error_InvalidValue
	};

	/*
	 *	CUnicodeFile::CUnicodeFile()
	 *		- default constructor
	 */
	CUnicodeFile();

	/*
	 *	CUnicodeFile::~CUnicodeFile()
	 *		- default destructor (close current file if necessary)
	 */
	~CUnicodeFile();

	/*
	 *	int CUnicodeFile::n_GetError()
	 *		- returns error code, corresponding with one of set bits of error flag and clears
	 *		  the bit (one of error_Unicode (unicode en/decoding error), error_Threshold (error
	 *		  generated by threshold unit with function set to wchar_Fail), error_Overflow
	 *		  (overflow occured while the overflow threshold unit was disabled), error_IO
	 *		  (generic input-output error) or error_InvalidValue (invalid value was used as
	 *		  function parameter - signifies either programming error or invalid data))
	 *		- in case no bits were set, returns error_NoError
	 */
	int n_GetError();

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
	/*
	 *	bool CUnicodeFile::EnableThreshold(int n_target, bool b_enable)
	 *		- if b_enable is true, enables (or if b_enable is false, disables) threshold unit
	 *		- by default both units are disabled
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- returns true in case valid unit was specified, otherwise returns false and raises
	 *		  error_InvalidValue flag
	 */
	bool EnableThreshold(int n_unit, bool b_enable);

	/*
	 *	bool CUnicodeFile::SetThresholdValue(int n_target, unsigned int n_threshold)
	 *		- sets value of threshold for selected unit to n_threshold
	 *		- n_threshold can be in range 0 to 0x10ffff (or 1114111 in decimal)
	 *		- default treshold value (for tresh_User unit) is 255
	 *		- n_target specifies threshold unit, currently can be only used with n_target set
	 *		  to thresh_User
	 *		- returns true in case valid unit and threshold was specified, otherwise returns false
	 *		  and raises error_InvalidValue flag
	 */
	bool SetThresholdValue(int n_unit, unsigned int n_threshold);

	/*
	 *	bool CUnicodeFile::SetThresholdFunction(int n_target, int n_function)
	 *		- sets function n_function to apply on character value in case threshold
	 *		  test on unit n_target fails
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- n_function specifies function, is one of wchar_Replace (character is replaced
	 *		  by some user-defined value), wchar_Translate (character is translated to escape
	 *		  sequence in format '\x????' where '?' are hexadecimal digits. their number can
	 *		  be set in range 2 to 8), wchar_Discard (character is discarded and the implementation
	 *		  acts as if it was never read; if necessary / suitable, another character is read
	 *		  to be returned)
	 *		  or wchar_Fail (the function fails)
	 *		- default function is wchar_Translate for both units
	 *		- note in case escape sequence is set too short for thresh_User and character with
	 *		  code too high to fit in is encountered, thresh_Overflow is triggered (in case
	 *		  it's disabled, current operation fails with error_Overflow)
	 *		- returns true in case valid unit and function was specified, otherwise returns false
	 *		  and raises error_InvalidValue flag
	 */
	bool SetThresholdFunction(int n_target, int n_function);

	/*
	 *	bool CUnicodeFile::SetThresholdReplacementChar(int n_target, unsigned int n_character)
	 *		- sets replacement char n_character for unit n_target
	 *		- n_character can be in range 0 to 0x10ffff (or 1114111 in decimal)
	 *		- default replacement char is '?' for both units
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- returns true in case valid unit and function was specified, otherwise returns false
	 *		  and raises error_InvalidValue flag
	 */
	bool SetThresholdReplacementChar(int n_target, unsigned int n_character);

	/*
	 *	bool CUnicodeFile::SetThresholdTranslationSize(int n_target, int n_size)
	 *		- sets threshold translation size n_size (in hexadecimal digits) unit n_target
	 *		- n_size can be in range 2 to 8 digits
	 *		- default size is 4 digits (16 bits) for both units
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- returns true in case valid unit and size was specified, otherwise returns false
	 *		  and raises error_InvalidValue flag
	 */
	bool SetThresholdTranslationSize(int n_target, int n_digit_num);

	/*
	 *	bool CUnicodeFile::b_ThresholdEnabled(int n_target)
	 *		- returns true in case threshold unit specified by n_target is enabled, otherwise false
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- in case invalid unit was specified returns always false and raises error_InvalidValue
	 *		  flag
	 */
	bool b_ThresholdEnabled(int n_target);

	/*
	 *	unsigned int CUnicodeFile::n_GetThresholdValue(int n_target)
	 *		- returns threshold value of unit specified by n_target
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- in case invalid unit was specified returns always 0 and raises error_InvalidValue
	 *		  flag
	 */
	unsigned int n_GetThresholdValue(int n_target);

	/*
	 *	int CUnicodeFile::n_GetThresholdFunction(int n_target)
	 *		- returns function set for threshold unit specified by n_target
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- in case invalid unit was specified returns always 0 and raises error_InvalidValue
	 *		  flag
	 */
	int n_GetThresholdFunction(int n_target);

	/*
	 *	unsigned int CUnicodeFile::n_GetThresholdReplacementChar(int n_target)
	 *		- returns replacement char set for threshold unit specified by n_target
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- in case invalid unit was specified returns always 0 and raises error_InvalidValue
	 *		  flag
	 */
	unsigned int n_GetThresholdReplacementChar(int n_target);

	/*
	 *	int CUnicodeFile::n_GetThresholdTranslationSize(int n_target)
	 *		- returns replacement char set for threshold unit specified by n_target
	 *		- n_target specifies threshold unit, can be either thresh_User for user controlled
	 *		  threshold unit or thresh_Overflow for overflow test unit
	 *		- in case invalid unit was specified returns always 0 and raises error_InvalidValue
	 *		  flag
	 */
	int n_GetThresholdTranslationSize(int n_target);
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING

	/*
	 *	bool CUnicodeFile::Open_Read(const char *p_s_filename, bool b_binary = true,
	 *		int n_default_encoding = code_ASCII);
	 *		- open file p_s_filename for reading. b_binary specifies wheter the file
	 *		  is to be used for reading in binary mode, n_default_encoding is used
	 *		  in case no BOM was found
	 *		- note unicode can contain such character codes that can have value of control
	 *		  characters so it can be necessary to open files in binary mode, even for text
	 *		  i/o (depends on your c standard library implementation)
	 *		- returns true on success, false on failure (raises error_InvalidValue flag in case
	 *		  invalid encoding was supplied, error_NoMemory flag in case cache buffer was not
	 *		  allocated and allocation failed or error_IO in case fopen() failed or writing BOM
	 *		  failed)
	 *		- returns false and raises error_InvalidOperation flag if file is opened already
	 *		  (don't want to hide Close() return value which may signify failure of delayed
	 *		  write (flush) ... which is good thing to know)
	 *		- note contents of n_default_encoding are checked only in case they were used
	 */
	bool Open_Read(const char *p_s_filename, bool b_binary = true,
		int n_default_encoding = code_ASCII);

	/*
	 *	bool CUnicodeFile::Open_Read(FILE *p_fr, int n_eof_position = -1, bool b_binary = true,
	 *		int n_default_encoding = code_ASCII);
	 *		- assign foreign file opened for reading. b_binary specifies wheter the file
	 *		  is to be used for reading in binary mode, n_default_encoding is used
	 *		  in case no BOM was found
	 *		- note unicode can contain such character codes that can have value of control
	 *		  characters so it can be necessary to open files in binary mode, even for text
	 *		  i/o (depends on your c standard library implementation)
	 *		- n_eof_position specifies position of the end of the file (-1 means physical
	 *		  end of the file; can be used for reading packed files such as files contained
	 *		  in not-compressed archives or resource-sections)
	 *		- current file position will be interpreted as beginning of file and it's assured
	 *		  no data from preceding parts of the file will be ever read
	 *		- returns true on success, false on failure (raises error_InvalidValue flag in case
	 *		  invalid encoding was supplied or null pointer to foreign file was suppiled,
	 *		  error_NoMemory flag in case cache buffer was not allocated and allocation failed
	 *		  or error_IO in case writing BOM failed)
	 *		- returns false and raises error_InvalidOperation flag if file is opened already
	 *		  (don't want to hide Close() return value which may signify failure of delayed
	 *		  write (flush) ... which is good thing to know)
	 *		- note contents of n_default_encoding are checked only in case they were used
	 */
	bool Open_Read(FILE *p_fr, int n_eof_position = -1, bool b_binary = true,
		int n_default_encoding = code_ASCII);

	/*
	 *	bool CUnicodeFile::Open_Write(const char *p_s_filename, bool b_binary = true,
	 *		int n_encoding = code_UTF_8, bool b_write_BOM = false);
	 *		- open file p_s_filename for writing. b_binary specifies wheter the file
	 *		  is to be used for writing in binary mode, n_encoding specifies encoding
	 *		  to be used, b_write_BOM specifies wheter to write BOM to the beginning
	 *		  of the file or not
	 *		- note unicode can contain such character codes that can have value of control
	 *		  characters so it can be necessary to open files in binary mode, even for text
	 *		  i/o (depends on your c standard library implementation)
	 *		- returns true on success, false on failure (raises error_InvalidValue flag in case
	 *		  invalid encoding was supplied, error_NoMemory flag in case cache buffer was not
	 *		  allocated and allocation failed or error_IO in case fopen() failed or writing BOM
	 *		  failed)
	 *		- returns false and raises error_InvalidOperation flag if file is opened already
	 *		  (don't want to hide Close() return value which may signify failure of delayed
	 *		  write (flush) ... which is good thing to know)
	 *		- note contents of n_default_encoding are checked only in case they were used
	 *		- note no BOM is written for code_ASCII even if required (but no error generated)
	 */
	bool Open_Write(const char *p_s_filename, bool b_binary = true,
		int n_encoding = code_UTF_8, bool b_write_BOM = false);

	/*
	 *	bool CUnicodeFile::Open_Write(FILE *p_fw, bool b_binary = true,
	 *		int n_encoding = code_UTF_8, bool b_write_BOM = false);
	 *		- assign foreign file opened for writing. b_binary specifies wheter the file
	 *		  is to be used for writing in binary mode, n_encoding specifies encoding
	 *		  to be used, b_write_BOM specifies wheter to write BOM to the beginning
	 *		  of the file or not
	 *		- note unicode can contain such character codes that can have value of control
	 *		  characters so it can be necessary to open files in binary mode, even for text
	 *		  i/o (depends on your c standard library implementation)
	 *		- current file position will be interpreted as beginning of file and it's assured
	 *		  no data from preceding parts of the file will be ever read
	 *		- as it is write mode, end of file is not constrained
	 *		- returns true on success, false on failure (raises error_InvalidValue flag in case
	 *		  invalid encoding was supplied or null pointer to foreign file was suppiled,
	 *		  error_NoMemory flag in case cache buffer was not allocated and allocation failed
	 *		  or error_IO in case writing BOM failed)
	 *		- returns false and raises error_InvalidOperation flag if file is opened already
	 *		  (don't want to hide Close() return value which may signify failure of delayed
	 *		  write (flush) ... which is good thing to know)
	 *		- note contents of n_default_encoding are checked only in case they were used
	 *		- note no BOM is written for code_ASCII even if required (but no error generated)
	 */
	bool Open_Write(FILE *p_fw, bool b_binary = true,
		int n_encoding = code_UTF_8, bool b_write_BOM = false);

	/*
	 *	bool CUnicodeFile::Close()
	 *		- returns true in case some file was opened, was successfuly closed and (in case
	 *		  cache buffer was not empty) flush was successfull; otherwise false
	 *		- in case foreign file was opened, inner state of CUnicodeFile is changed only
	 *		  and fclose() is not called
	 *		- in case no file was opened, raises error_InvalidOperation flag
	 *		- in case flush was not successful, error_IO flag is raised
	 */
	bool Close();

	/*
	 *	bool CUnicodeFile::Flush()
	 *		- in read mode clears translated character cache (contains up to 10 characters
	 *		  of translated wide character hexadecimal code)
	 *		- in write mode writes output buffer to file and clears output buffer
	 *		- in read mode returns true in case file was opened
	 *		- in write mode returns true in case file was opened and there was either no data
	 *		  to flush or flush was successful (in case it was not, raises error_IO flag)
	 *		- returns false and raises error_InvalidOperation flag in case no file was opened
	 */
	bool Flush();

	/*
	 *	bool CUnicodeFile::b_Opened() const
	 *		- returns true in case some file is opened, otherwise false
	 *		- does not raise any error flag
	 */
	bool b_Opened() const;

	/*
	 *	bool CUnicodeFile::b_Contains_BOM() const
	 *		- returns true in case current file contains BOM (byte-order mark)
	 *		- always returns false in case no file was opened
	 *		- does not raise any error flag
	 */
	bool b_Contains_BOM() const;

	/*
	 *	int CUnicodeFile::n_Encoding() const
	 *		- returns current file encoding (code_ASCII, code_UTF_8, code_UTF_16_LE or
	 *		  code_Unsupported)
	 *		- always returns code_Unsupported in case no file was opened
	 *		- does not raise any error flag
	 */
	int n_Encoding() const;

	/*
	 *	bool CUnicodeFile::SetEncoding(int n_encoding)
	 *		- sets encoding to n_encoding (one of code_ASCII, code_UTF_8 or code_UTF_16_LE)
	 *		- can be set only in read mode and in case BOM was not found only (scenario of XML
	 *		  file where encoding is determined by reading first line of file which is always
	 *		  ascii)
	 *		- returns true on success, returns false on failure. raises error_InvalidOperation
	 *		  flag in case no file was opened, it was opened for writing or there was BOM found,
	 *		  raises error_InvalidValue flag in case invalid encoding is supplied in n_encoding
	 */
	bool SetEncoding(int n_encoding);

	/*
	 *	bool CUnicodeFile::Seek(int n_offset, int n_mode)
	 *		- set file to certain position
	 *		- n_offset is offset in bytes (note UTF uses codes with variable length)
	 *		- n_mode defines where to start (one of seek_Relative, seek_Beginning, seek_End)
	 *		- n_offset is offset in bytes (careful to use it - setting file pointer to the
	 *		  middle of multiple-byte character would result in it's bad decoding (after a few
	 *		  more failures decoder should recover; amount depends on encoding and file position))
	 *		- returns true on success, false on failure (raises error_InvalidOperation flag in case
	 *		  no file was opened, error_InvalidValue flag on invalid offset, error_InvalidOperation
	 *		  flag on invalid mode or error_IO flag if fseek() failed)
	 *		- note seeking from seek_Beginning effectively skips BOF mark in case it's present
	 *		- note seeking clears buffer for text-input escape sequences which couldn't
	 *		  be returned in one piece in last call to one of text input functions
	 */
	bool Seek(int n_offset, int n_mode);

	/*
	 *	int CUnicodeFile::n_Tell()
	 *		- return position in bytes file pointer points to
	 *		  (note UTF uses codes with variable length)
	 *		- position is measurred from the end of BOM mark if present and from the position
	 *		  file pointer pointed to when passed to Open_?(FILE *, ...) in case that was how
	 *		  file pointer was acquired (i.e. after Open_?(...) call to n_Tell() always returns 0)
	 *		- returns -1 in case no file is opened (raises error_InvalidOperation flag) or i/o
	 *		  error ocurred (raises error_IO flag)
	 */
	int n_Tell();

	/*
	 *	template<class CUnsignedIntType>
	 *	int CUnicodeFile::n_Read(CUnsignedIntType *p_dest, int n_count)
	 *		- binary mode i/o. read n_count elements into p_dest array
	 *		- returns number of elements that were really read, -1 on failure (raises
	 *		  error_InvalidOperation flag in case file is not opened or in case file is opened
	 *		  for writing), (0 can occur if reading past the end of the file which technically
	 *		  is not failure - call n_GetError())
	 *		- it may raise error_IO on i/o error, error_Treshold on triggering treshold unit
	 *		  with wchar_Fail specified or error_Overflow in case type of p_dest isn't big enough
	 *		  to accomodate character ascii and no (suitable) character transformation is enabled
	 *		- CUnsignedIntType is unsigned char, unsigned short or unsigned int, depending
	 *		  on range of expected characters
	 *		- note binary ops clears buffer for text-input escape sequences which couldn't
	 *		  be returned in one piece in last call to one of text input functions
	 */
	template<class CUnsignedIntType>
	int n_Read(CUnsignedIntType *p_dest, int n_count)
	{
		__FuncGuard("CUnicodeFile::n_Read");

		if(!m_p_file || m_b_writing) {
			Raise(error_InvalidOperation);
			return -1;
		}
		// has to be opened for reading

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
		m_n_string_buffer_used = 0;
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING

		int n_read_counter = 0;

		if(m_n_encoding == code_UTF_8) {
			for(const CUnsignedIntType *p_end = p_dest + n_count; p_dest < p_end; ++ n_read_counter) {
				if(!GetWideChar_UTF_8(*p_dest ++)) // t_odo - how to handle this? compare error flag with zero and then either break or return? yes. even if we break, less characters will be read so user would call n_GetError()
					break;
			}
			// UTF-8 code path
		} else if(m_n_encoding == code_UTF_16_LE) {
			for(const CUnsignedIntType *p_end = p_dest + n_count; p_dest < p_end; ++ n_read_counter) {
				if(!GetWideChar_UTF_16_LE(*p_dest ++))
					break;
			}
			// UTF-16-LE code path
		} else /*if(m_n_encoding == code_ASCII)*/ {
			for(const CUnsignedIntType *p_end = p_dest + n_count; p_dest < p_end; ++ n_read_counter) {
				uint8_t n_byte;
				if(!ReadByte(n_byte))
					break;
				*p_dest ++ = n_byte;
			}
			// ASCII code path
		}
		// read character data in the loop

		return n_read_counter;
	}

	/*
	 *	template<class CUnsignedIntType>
	 *	int CUnicodeFile::n_Write(const CUnsignedIntType *p_src, int n_count)
	 *		- binary mode i/o. write n_count elements of p_src array
	 *		- returns number of elements (not bytes; note UTF uses codes with variable length)
	 *		  that were sent to be written (note the value propably doesn't match number of
	 *		  bytes really written because of delayed writing), returns -1 and raises
	 *		  error_InvalidOperation flag in case file is not opened or in case file is opened
	 *		  for reading
	 *		- error_IO or error_Unicode flags can be raised in case function doesn't process all
	 *		  the characters
	 *		- CUnsignedIntType is unsigned char, unsigned short or unsigned int, depending
	 *		  on range of expected characters
	 *		- note binary ops clears buffer for text-input escape sequences which couldn't
	 *		  be returned in one piece in last call to one of text input functions
	 */
	template<class CUnsignedIntType>
	int n_Write(const CUnsignedIntType *p_src, int n_count)
	{
		__FuncGuard("CUnicodeFile::n_Write");

		if(!m_p_file || !m_b_writing) {
			Raise(error_InvalidOperation);
			return -1;
		}
		// has to be opened for writing

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
		m_n_string_buffer_used = 0;
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING

		int n_write_counter = 0;

		if(m_n_encoding == code_UTF_8) {
			for(const CUnsignedIntType *p_end = p_src + n_count; p_src < p_end; ++ n_write_counter) {
				if(!PutWideChar_UTF_8(*p_src ++))
					break;
			}
			// UTF-8 code path
		} else if(m_n_encoding == code_UTF_16_LE) {
			for(const CUnsignedIntType *p_end = p_src + n_count; p_src < p_end; ++ n_write_counter) {
				if(!PutWideChar_UTF_16_LE(*p_src ++))
					break;
			}
			// UTF-16-LE code path
		} else /*if(m_n_encoding == code_ASCII)*/ {
			for(const CUnsignedIntType *p_end = p_src + n_count; p_src < p_end; ++ n_write_counter) {
				if(!WriteByte((uint8_t)*p_src ++))
					break;
			}
			// ASCII path
		}
		// write character data in the loop

		return n_write_counter;
	}

	/*
	 *	bool CUnicodeFile::GetLine(char *p_dest, int n_max_length)
	 *		- text mode i/o. return string, containing characters up to 0xff
	 *		- characters above 0xff are written in their hexadecimal form (ie. \x??????),
	 *		  backslash characters '\' are duplicated "\\"; can be disabled by EnableWideChars()
	 *		- newline '\n' character is contained in returned string
	 *		- p_dest is destination buffer for zero-terminated string
	 *		- n_max_length is maximal length of data, including terminating zero
	 *		- uses some cache to return parts of escape-sequences in case they do not fit whole
	 *		  into destination buffer. (escape sequences will be properly returned with next call
	 *		  to the next text-mode input function prior to reading more bytes from file)
	 *		- returns true on success or returns false and raises error_InvalidOperation flag
	 *		  in case file is not opened or in case file is opened for writing or raises
	 *		  error_Treshold flag on triggering treshold unit with wchar_Fail specified or
	 *		  error_Overflow in case type of p_dest isn't big enough to accomodate character ascii
	 *		  and no (suitable) character transformation is enabled)
	 *		- it may raise error_IO flag on i/o error or error_Unicode flag on invalid unicode
	 *		  sequence while returning true but not reading in full length of buffer
	 */
	bool GetLine(char *p_dest, int n_max_length);

	/*
	 *	int CUnicodeFile::n_GetChar()
	 *		- text mode i/o. return a single character of string, containing characters up to 0xff
	 *		- characters above 0xff are written in their hexadecimal form (ie. \x??????),
	 *		  backslash characters '\' are duplicated "\\"; can be disabled by EnableWideChars()
	 *		- uses some cache to return escape-sequences so be careful when using it in conjunction
	 *		  with the other functions (escape sequences will be properly returned with next call
	 *		  to the next text-mode input function prior to reading more bytes from file)
	 *		- returns character value or returns -1 and raises error_InvalidOperation flag
	 *		  in case file is not opened or in case file is opened for writing or raises error_IO
	 *		  flag on i/o error, error_Treshold on triggering treshold unit with wchar_Fail
	 *		  specified or error_Overflow on triggering tresh_User unit while no (suitable)
	 *		  character transformation is enabled and tresh_Overflow unit is disabled or raises
	 *		  error_Unicode flag on invalid unicode sequence
	 */
	int n_GetChar();

	/*
	 *	template <class CGetCharFunctor>
	 *	CGetCharFunctor CUnicodeFile::t_ForEachChar(CGetCharFunctor t_get_char)
	 *		- text mode i/o. read all chars from the current position to the end of file
	 *		  (in case part of escape-sequence was left in the buffer by last call to one of text
	 *		  input functions which was not able to return it whole, it's output prior to reading
	 *		  any actual bytes)
	 *		- every char is passed to CGetCharFunctor as the only parameter
	 *		- returns t_get_char, might raise error_InvalidOperation flag in case file is not
	 *		  opened or in case file is opened for writing, can raise error_Treshold flag
	 *		  on triggering treshold unit with wchar_Fail specified or error_Overflow in case type
	 *		  of p_dest isn't big enough to accomodate character ascii and no (suitable)
	 *		  character transformation is enabled)
	 *		- it may raise error_IO flag on i/o error or error_Unicode flag on invalid unicode
	 *		  sequence while returning t_get_char but not reading in full length of file
	 */
	template <class CGetCharFunctor>
	CGetCharFunctor t_ForEachChar(CGetCharFunctor t_get_char)
	{
		__FuncGuard("CUnicodeFile::t_ForEachChar");

		if(!m_p_file || m_b_writing) {
			Raise(error_InvalidOperation);
			return t_get_char;
		}
		// has to be opened for reading

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
		if(m_n_string_buffer_used) {
			for(char *p_cur = m_p_string_buffer, *p_end = m_p_string_buffer +
			   m_n_string_buffer_used; p_cur < p_end; ++ p_cur) {
				/*if(!(bool)*/t_get_char(*p_cur)/*)
					return -2*/;
			}
			m_n_string_buffer_used = 0;
		}
		// in case something was in the buffer, eat it up first
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING

		for(int n = 0;; ++ n) {
			int n_char;
			if(m_n_encoding == code_UTF_8) {
				if(!GetWideChar_UTF_8(n_char))
					break;
			} else if(m_n_encoding == code_UTF_16_LE) {
				if(!GetWideChar_UTF_16_LE(n_char))
					break;
			} else /*if(m_n_encoding == code_ASCII)*/ {
				uint8_t n_tmp;
				if(!ReadByte(n_tmp))
					break;
				n_char = n_tmp;
			} // t_odo - add branch for code_ASCII (and above as well)

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
			unsigned int n_char2, *p_char2 = &n_char2;
			if(!Process_WideChars(p_char2, p_char2 + 1, n_char))
				return t_get_char;
			// wide char translation

			if(p_char2 == &n_char2) {
				_ASSERTE(!m_n_string_buffer_used);
				continue;
			}
			// see if character was discarded

			t_get_char(n_char2);
			// eat it

			if(m_n_string_buffer_used) {
				for(char *p_cur = m_p_string_buffer, *p_end = m_p_string_buffer +
				   m_n_string_buffer_used; p_cur < p_end; ++ p_cur) {
					/*if(!(bool)*/t_get_char(*p_cur)/*)
						return -2*/;
				}
				m_n_string_buffer_used = 0;
			}
			// eat the others in case this one was wide
#else // __UNICODE_FILE_ENABLE_CHAR_PROCESSING
			t_get_char(n_char);
			// eat it
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING
		}

		return t_get_char;
	}

	/*
	 *	bool CUnicodeFile::PutChar(unsigned int n_char)
	 *		- text mode i/o. write a single character
	 *		- returns true in case of success, false on failure
	 *		- raises error_InvalidOperation in case file was not opened or in case file was opened
	 *		  for reading, raises error_Unicode flag in case n_char is not valid unicode character,
	 *		  raises error_IO flag in case of i/o error
	 */
	bool PutChar(unsigned int n_char);

	/*
	 *	template<class CUnsignedIntType>
	 *	int CUnicodeFile::n_WriteString(const CUnsignedIntType *p_string)
	 *		- text mode i/o. write zero terminated string
	 *		- in case tresh_User is enabled and wchar_Translate function is specified, every
	 *		  backslash character must be followed either by another one (single backslash
	 *		  character is written only) or by escape sequence which must be as long as specified
	 *		  for character translation size for tresh_User
	 *		- in case tresh_User is enabled and wchar_Replace is specified, every character greater
	 *		  than specified threshold is replaced by specified replacement character
	 *		- in case tresh_User is enabled and wchar_Discard is specified, every character greater
	 *		  than specified threshold is skipped
	 *		- in case tresh_User is enabled and wchar_Fail is specified, every character greater
	 *		  than specified threshold makes function fail and raise error_Threshold flag
	 *		- returns number of bytes written (in case of success) or -1 on failure
	 *		- raises error_InvalidOperation in case file was not opened or in case file was opened
	 *		  for reading, raises error_Unicode flag in case n_char is not valid unicode character,
	 *		  raises error_IO flag in case of i/o error, raises error_Threshold flag in case
	 *		  treshold unit tresh_User is enabled, was triggered by backslash and error_Threshold
	 *		  was specified for tresh_User or raises error_InvalidValue flag in case treshold unit
	 *		  tresh_User is enabled, was triggered by backslash and following escape sequecnce was
	 *		  containing invalid hexadecimal characters (anything else than [0-9a-fA-F])
	 */
	template<class CUnsignedIntType>
	int n_WriteString(const CUnsignedIntType *p_string)
	{
		__FuncGuard("CUnicodeFile::n_WriteString");

		int n_byte_num = 0;

		for(; *p_string; ++ p_string) {
			unsigned int n_char;
#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
			if(m_t_wide_char_proc.b_enable) {
				if(*p_string == '\\' && m_t_wide_char_proc.n_mode == wchar_Translate) {
					if(*(++ p_string) == '\\')
						n_char = '\\'; // double backslash
					else if(*p_string == 'x') {
						n_char = 0;
						++ p_string;
						for(const CUnsignedIntType *p_end = p_string + m_t_wide_char_proc.n_format_digit_num;
						   p_string < p_end; ++ p_string) {
							n_char <<= 4;
							if(*p_string >= '0' && *p_string <= '9')
								n_char |= *p_string - '0';
							else if(*p_string >= 'a' && *p_string <= 'f')
								n_char |= *p_string - 'a' + 10;
							else if(*p_string >= 'A' && *p_string <= 'F')
								n_char |= *p_string - 'A' + 10;
							else {
								Raise(error_InvalidValue);
								return -1;
							}
						}
						// decode m_n_wide_char_digit_num-char hexadecimal sequence

						-- p_string; // step back, we would miss the next character
					} else
						return -1; // malformed escape sequence
				} else if(*p_string > m_t_wide_char_proc.n_threshold) {
					if(m_t_wide_char_proc.n_mode == wchar_Replace)
						n_char = m_t_wide_char_proc.n_replacement_char; // replace character
					else if(m_t_wide_char_proc.n_mode == wchar_Discard)
						continue; // skip character
					else /*if(m_t_wide_char_proc.n_mode == wchar_Fail)*/ {
						Raise(error_Threshold);
						return -1; // fail
					}
				} else
					n_char = *p_string;
			} else	
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING
				n_char = *p_string;
			// take care of wide char replacement / transformation

			if(m_n_encoding == code_UTF_8) {
				if(!PutWideChar_UTF_8(n_char))
					return -1;
				n_byte_num += (n_char < 0x80)? 1 : ((n_char < 0x800)? 2 : ((n_char < 0x10000)? 3 : 4));
			} else if(m_n_encoding == code_UTF_16_LE) {
				if(!PutWideChar_UTF_16_LE(n_char))
					return -1;
				n_byte_num += (n_char < 0x10000)? 2 : 4;
			} else /*if(m_n_encoding == code_ASCII)*/ {
				if(!WriteByte((uint8_t)n_char))
					return -1;
				n_byte_num += 1;
			}
		}

		return n_byte_num;
	}

protected:
	inline void Raise(int n_error)
	{
		__FuncGuard("CUnicodeFile::Raise");

		_ASSERTE(n_error >= (int)error_Unicode && n_error <= (int)error_InvalidValue);
		m_n_error_state |= 1 << (n_error - 1);
	}

	bool Alloc_FileBuffer();
	void Delete_FileBuffer();
	bool Find_BOM();
	int n_Flood_Buffer();
	inline bool ReadByte(uint8_t &r_n_byte);
	inline bool ReadMBS(uint8_t *p_dest, int n_size);
	inline bool WriteByte(uint8_t n_byte);

	template <class CUnsignedIntType>
	inline bool GetWideChar_UTF_8(CUnsignedIntType &r_n_result)
	{
		__FuncGuard("CUnicodeFile::GetWideChar_UTF_8");

		uint8_t n_byte;
		if(!ReadByte(n_byte))
			return false;
		// get byte

		if((n_byte & 0x80) == 0) { // 0x0*******
			r_n_result = (CUnsignedIntType)n_byte;
			return true;
		}
		// a single ascii (7-bit) character (propably most of any character stream)

		if((n_byte & 0xe0) == 0xc0) { // 0x110***** 0x10******
			uint8_t n_byte2;
			if(!ReadByte(n_byte2) || (n_byte2 & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x1f) << 6) | (n_byte2 & 0x3f);

			if(n_character < 0x80) { // invalid range for two-byte character
				Raise(error_Unicode);
				return false;
			}

			r_n_result = n_character;
			// two-byte character

			if(r_n_result == n_character)
				return true;

			Raise(error_Overflow);
			return false;
		} else if((n_byte & 0xf0) == 0xe0) { // 0x1110**** 0x10****** 0x10******
			uint8_t p_byte[2];
			if(!ReadMBS(p_byte, 2) || (p_byte[0] & 0xc0) != 0x80 || (p_byte[1] & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x0f) << 12) |
				((p_byte[0] & 0x3f) << 6) | (p_byte[1] & 0x3f);

			if((n_character >= 0xd800 && n_character <= 0xdfff) || n_character < 0x800) {
				Raise(error_Unicode);
				return false;
			}
			// UTF-16 surrogates - forbidden as they are non-characters,
			// check range for three-byte character

			r_n_result = n_character;
			// three-byte character

			if(r_n_result == n_character)
				return true;

			Raise(error_Overflow);
			return false;
		} else if((n_byte & 0xf8) == 0xf0) { // 0x11110*** 0x10****** 0x10****** 0x10******
			uint8_t p_byte[3];
			if(!ReadMBS(p_byte, 3) || (p_byte[0] & 0xc0) != 0x80 ||
			   (p_byte[1] & 0xc0) != 0x80 || (p_byte[2] & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x07) << 18) | ((p_byte[0] & 0x3f) << 12) |
				((p_byte[1] & 0x3f) << 6) | (p_byte[2] & 0x3f);

			if(n_character > 0x10ffff || n_character < 0x10000) {
				Raise(error_Unicode);
				return false;
			}
			// check range for four-byte character

			r_n_result = n_character;
			// four-byte character

			if(r_n_result == n_character)
				return true;

			Raise(error_Overflow);
			return false;			
		} else {
			Raise(error_Unicode);
			return false; // invalid number of bytes
		}
	}

#ifndef __GNUC__
	template <>
	inline bool GetWideChar_UTF_8<int>(int &r_n_result)
	{
		__FuncGuard("CUnicodeFile::GetWideChar_UTF_8");

		uint8_t n_byte;
		if(!ReadByte(n_byte))
			return false;
		// get byte

		if((n_byte & 0x80) == 0) { // 0x0*******
			r_n_result = n_byte;
			return true;
		}
		// a single ascii (7-bit) character (propably most of any character stream)

		if((n_byte & 0xe0) == 0xc0) { // 0x110***** 0x10******
			uint8_t n_byte2;
			if(!ReadByte(n_byte2) || (n_byte2 & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x1f) << 6) | (n_byte2 & 0x3f);

			if(n_character < 0x80) { // invalid range for two-byte character
				Raise(error_Unicode);
				return false;
			}

			r_n_result = n_character;
			// two-byte character

			return true;
		} else if((n_byte & 0xf0) == 0xe0) { // 0x1110**** 0x10****** 0x10******
			uint8_t p_byte[2];
			if(!ReadMBS(p_byte, 2) || (p_byte[0] & 0xc0) != 0x80 || (p_byte[1] & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x0f) << 12) |
				((p_byte[0] & 0x3f) << 6) | (p_byte[1] & 0x3f);

			if((n_character >= 0xd800 && n_character <= 0xdfff) || n_character < 0x800) {
				Raise(error_Unicode);
				return false;
			}
			// UTF-16 surrogates - forbidden as they are non-characters,
			// check range for three-byte character

			r_n_result = n_character;
			// three-byte character

			return true;
		} else if((n_byte & 0xf8) == 0xf0) { // 0x11110*** 0x10****** 0x10****** 0x10******
			uint8_t p_byte[3];
			if(!ReadMBS(p_byte, 3) || (p_byte[0] & 0xc0) != 0x80 ||
			   (p_byte[1] & 0xc0) != 0x80 || (p_byte[2] & 0xc0) != 0x80) {
				Raise(error_Unicode);
				return false;
			}

			int n_character = ((n_byte & 0x07) << 18) | ((p_byte[0] & 0x3f) << 12) |
				((p_byte[1] & 0x3f) << 6) | (p_byte[2] & 0x3f);

			if(n_character > 0x10ffff || n_character < 0x10000) {
				Raise(error_Unicode);
				return false;
			}
			// check range for four-byte character

			r_n_result = n_character;
			// four-byte character

			return true;		
		} else {
			Raise(error_Unicode);
			return false; // invalid number of bytes
		}
	}
#endif // __GNUC__

	template <class CUnsignedIntType>
	inline bool GetWideChar_UTF_16_LE(CUnsignedIntType &r_n_result)
	{
		__FuncGuard("CUnicodeFile::GetWideChar_UTF_16_LE");

		uint16_t n_word;
		if(!ReadMBS((uint8_t*)&n_word, 2))
			return false; // t_odo - see if the endianness is right - it is
		// get word

		if((n_word >> 10) != 0x36) {
			if(n_word >= 0xfffe || (n_word >= 0xfdd0 && n_word <= 0xfdef)) {
				Raise(error_Unicode);
				return false;
			}
			// check for non-characters

			r_n_result = (CUnsignedIntType)n_word;

			if(r_n_result == n_word)
				return true;

			Raise(error_Overflow);
			return false;
		}
		// it was in the range for single characters (not a high surrogate)

		uint16_t n_word2;
		if(!ReadMBS((uint8_t*)&n_word2, 2))
			return false; // t_odo - see if the endianness is right - it is
		// get word

		if((n_word2 >> 10) != 0x37) {
			Raise(error_Unicode);
			return false;
		}
		// word2 has to be low surrogate

		/*if((n_word2 == 0xdffe || n_word2 == 0xdfff) && ((n_word & 0xf03f) == 0xd03f &&
		   (n_word & 0x0fc0) >= 0x0800 && (n_word & 0x0fc0) <= 0x0bc0)
			return -2;
		// check for UTF-16 non-characters: // todo - check if word order is right
		// D83F DFF*
		// D87F DFF*
		// D8BF DFF*
		// D8FF DFF*
		// D93F DFF*
		// D97F DFF*
		// ...
		// DBBF DFF*
		// DBFF DFF*
		// * = E or F */
		// according to the specs, those non-characters might be assigned in later
		// versions - policy is to let them trough, but avoid generating them

		unsigned int n_result2 = (((n_word & 0x3ff) << 10) | (n_word2 & 0x3ff)) + 0x10000;
		// take 10 low bits of first word to be used as hi-part
		// and 10 low bytes of the other word to be used as lo-part
		// do not forget to add the displacement

		/*if((n_result2 & 0xffff) >= 0xfffe)
			return -2;*/
		// effectively the same check for UTF-16 non-characters as the one above
		// according to the specs, those non-characters might be assigned in later
		// versions - policy is to let them trough, but avoid generating them

		if(/*n_result2 < 0x10000 ||*/ n_result2 > 0x10ffff) {
			Raise(error_Unicode);
			return false;
		}
		// can't use surrogate pair for encding value that'd fit into a single character
		// f_ixme - could that happen? no, but it's good place to perform maximal value check

		r_n_result = (CUnsignedIntType)n_result2;

		if((unsigned int)r_n_result == n_result2)
			return true;

		Raise(error_Overflow);
		return false;
	}

#ifndef __GNUC__
	template <>
	inline bool GetWideChar_UTF_16_LE<int>(int &r_n_result)
	{
		
		__FuncGuard("CUnicodeFile::GetWideChar_UTF_16_LE");

		uint16_t n_word;
		if(!ReadMBS((uint8_t*)&n_word, 2))
			return false; // t_odo - see if the endianness is right - it is
		// get word

		if((n_word >> 10) != 0x36) {
			if(n_word >= 0xfffe || (n_word >= 0xfdd0 && n_word <= 0xfdef)) {
				Raise(error_Unicode);
				return false;
			}
			// check for non-characters

			r_n_result = n_word;

			return true;
		}
		// it was in the range for single characters (not a high surrogate)

		uint16_t n_word2;
		if(!ReadMBS((uint8_t*)&n_word2, 2))
			return false; // t_odo - see if the endianness is right - it is
		// get word

		if((n_word2 >> 10) != 0x37) {
			Raise(error_Unicode);
			return false;
		}
		// word2 has to be low surrogate

		//if((n_word2 == 0xdffe || n_word2 == 0xdfff) && ((n_word & 0xf03f) == 0xd03f &&
		//   (n_word & 0x0fc0) >= 0x0800 && (n_word & 0x0fc0) <= 0x0bc0)
		//	return -2;
		// check for UTF-16 non-characters: // todo - check if word order is right
		// D83F DFF*
		// D87F DFF*
		// D8BF DFF*
		// D8FF DFF*
		// D93F DFF*
		// D97F DFF*
		// ...
		// DBBF DFF*
		// DBFF DFF*
		// * = E or F 
		// according to the specs, those non-characters might be assigned in later
		// versions - policy is to let them trough, but avoid generating them

		unsigned int n_result2 = (((n_word & 0x3ff) << 10) | (n_word2 & 0x3ff)) + 0x10000;
		// take 10 low bits of first word to be used as hi-part
		// and 10 low bytes of the other word to be used as lo-part
		// do not forget to add the displacement

		//if((n_result2 & 0xffff) >= 0xfffe)
		//	return -2;
		// effectively the same check for UTF-16 non-characters as the one above
		// according to the specs, those non-characters might be assigned in later
		// versions - policy is to let them trough, but avoid generating them

		if(n_result2 < 0x10000 || n_result2 > 0x10ffff) {
			Raise(error_Unicode);
			return false;
		}
		// can't use surrogate pair for encding value that'd fit into a single character
		// f_ixme - could that happen? no, but it's good place to perform maximal value check

		r_n_result = n_result2;

		return true;
	}
#endif // __GNUC__

	template <class CUnsignedIntType>
	inline bool PutWideChar_UTF_8(CUnsignedIntType n_char)
	{
		__FuncGuard("CUnicodeFile::PutWideChar_UTF_8");

		_ASSERTE(n_char > 0);
		if(n_char < 0x80)
			return WriteByte(n_char);
		else if(n_char < 0x800)
			return WriteByte(0xc0 | (n_char >> 6)) && WriteByte(0x80 | (n_char & 0x3f));
		else if(n_char < 0x10000) {
			if(n_char >= 0xd800 && n_char <= 0xdfff) {
				Raise(error_Unicode);
				return false;
			}
			// we should not encode UTF-16 surrogates into UTF-8 stream as they are non-characters

			if((n_char & 0xffff) >= 0xfffe || (n_char >= 0xffd0 && n_char <= 0xfdef)) {
				Raise(error_Unicode);
				return false;
			}
			// more invalid chracters

			return WriteByte(0xe0 | (n_char >> 12)) && WriteByte(0x80 | ((n_char >> 6) & 0x3f)) &&
				WriteByte(0x80 | (n_char & 0x3f));
		} else if(n_char <= 0x10ffff) {
			return WriteByte(0xf0 | (n_char >> 18)) && WriteByte(0x80 | ((n_char >> 12) & 0x3f)) &&
				WriteByte(0x80 | ((n_char >> 6) & 0x3f)) && WriteByte(0x80 | (n_char & 0x3f));
		} else
			Raise(error_Unicode);

		return false;
	}

	template <class CUnsignedIntType>
	inline bool PutWideChar_UTF_16_LE(CUnsignedIntType n_char)
	{
		__FuncGuard("CUnicodeFile::PutWideChar_UTF_16_LE");

		_ASSERTE(n_char > 0);
		if(n_char > 0x10ffff) {
			Raise(error_Unicode);
			return false;
		}
		// invalid range

		if((n_char & 0xffff) >= 0xfffe || (n_char >= 0xffd0 && n_char <= 0xfdef)) {
			Raise(error_Unicode);
			return false;
		}
		// invalid chracters

		if(n_char < 0x10000) {
			if(!WriteByte((uint8_t)n_char) || !WriteByte((uint8_t)(n_char >> 8)))
				return false;
			return true;
		}
		// character is in range

		n_char -= 0x10000;
		uint16_t n_word = 0xd800 | (n_char >> 10);
		if(!WriteByte((uint8_t)n_word) || !WriteByte((uint8_t)(n_word >> 8)))
			return false;
		n_word = 0xdc00 | (n_char & 0x3ff);
		if(!WriteByte((uint8_t)n_word) || !WriteByte((uint8_t)(n_word >> 8)))
			return false;
		// write as pair of surrogates

		return true;
	}

#ifdef __UNICODE_FILE_ENABLE_CHAR_PROCESSING
	/*
	 *	template <class CUnsignedIntType>
	 *	bool Process_WideChars(CUnsignedIntType *&p_dest, const CUnsignedIntType *p_end,
	 *		unsigned int n_char)
	 *		- take source char n_char and copy it's proper translation
	 *		  into p_dest; p_end is end of buffer, the function is never going to be called
	 *		  with empty buffer
	 *		- returns true on success, false on failure (Overflow / wchar_Fail)
	 */
	template <class CUnsignedIntType>
	bool Process_WideChars(CUnsignedIntType *&p_dest,
		const CUnsignedIntType *p_end, unsigned int n_char)
	{
		__FuncGuard("CUnicodeFile::Process_WideChars");

		_ASSERTE(p_dest < p_end);

		do {
			if(m_t_wide_char_proc.b_enable && n_char > m_t_wide_char_proc.n_threshold) {
				if(m_t_wide_char_proc.n_mode == wchar_Translate) {
					if(n_char > (unsigned int)(~0U >> (sizeof(int) * 8 -
					   m_t_wide_char_proc.n_format_digit_num * 4)))
						break;
					// see if we'd fit into designed number of digits (overflow as well)

					*p_dest ++ = '\\';

					sprintf(m_p_string_buffer, m_t_wide_char_proc.p_s_format_string, n_char);
					m_n_string_buffer_used = m_t_wide_char_proc.n_format_digit_num + 1;
					// output it to the string buffer

					if(p_dest + m_t_wide_char_proc.n_format_digit_num + 1 < p_end) {
						p_end = p_dest + m_t_wide_char_proc.n_format_digit_num + 1;
						for(char *p_src = m_p_string_buffer; p_dest < p_end;)
							*p_dest ++ = (CUnsignedIntType)(*p_src ++);
						m_n_string_buffer_used = 0;
						// copy the whole string buffer to output
						return true;
					} else {
						if(p_dest < p_end) {
							int n_length = p_end - p_dest;
							for(char *p_src = m_p_string_buffer; p_dest < p_end;)
								*p_dest ++ = (CUnsignedIntType)(*p_src ++);
							m_n_string_buffer_used -= n_length;
							memcpy(m_p_string_buffer, m_p_string_buffer + n_length,
								m_n_string_buffer_used);
						}
						// output the whole sequence to the string buffer, use part, keep the rest
						return true;
					}
				} else if(m_t_wide_char_proc.n_mode == wchar_Replace) {
					*p_dest ++ = m_t_wide_char_proc.n_replacement_char;
					return true;
				} else if(m_t_wide_char_proc.n_mode == wchar_Discard)
					return true;
				else /*if(m_t_wide_char_proc.n_mode == wchar_Fail)*/ {
					_ASSERTE(m_t_wide_char_proc.n_mode == wchar_Fail);
					Raise(error_Threshold);
					return false;
				}
			}

			if(n_char == '\\' &&
			   ((m_t_wide_char_proc.b_enable && m_t_wide_char_proc.n_mode == wchar_Translate) ||
			   (m_t_overflow_proc.b_enable && m_t_overflow_proc.n_mode == wchar_Translate))) {
				*p_dest ++ = '\\';
				if(p_dest < p_end)
					*p_dest ++ = '\\';
				else {
					m_p_string_buffer[0] = '\\';
					m_n_string_buffer_used = 1;
				}
				return true;
			}
			// duplicate backslash characters if wide char translation is enabled

			_ASSERTE('\\' <= maxvalueof(CUnsignedIntType));
			if(n_char <= maxvalueof(CUnsignedIntType)) {
				*p_dest ++ = n_char;
				return true;
			}
			// check for overflow
		} while(0);

		// here it's sure overflow is going to occur

		if(!m_t_overflow_proc.b_enable) {
			Raise(error_Overflow);
			return false;
		}
		// overflow handler has to be enabled

		if(m_t_overflow_proc.n_mode == wchar_Translate) {
			if(n_char > (unsigned int)(~0U >> (sizeof(int) * 8 -
			   m_t_overflow_proc.n_format_digit_num * 4))) {
				Raise(error_Overflow);
				return false;
			}
			// we didn't escape unhandled overflow, not enough digits enabled

			*p_dest ++ = '\\';

			sprintf(m_p_string_buffer, m_t_overflow_proc.p_s_format_string, n_char);
			m_n_string_buffer_used = m_t_overflow_proc.n_format_digit_num + 1;
			// output it to the string buffer

			if(p_dest + m_t_overflow_proc.n_format_digit_num + 1 < p_end) {
				p_end = p_dest + m_t_overflow_proc.n_format_digit_num + 1;
				for(char *p_src = m_p_string_buffer; p_dest < p_end;)
					*p_dest ++ = (CUnsignedIntType)(*p_src ++);
				m_n_string_buffer_used = 0;
				// copy the whole string buffer to output
				return true;
			} else {
				if(p_dest < p_end) {
					int n_length = p_end - p_dest;
					for(char *p_src = m_p_string_buffer; p_dest < p_end;)
						*p_dest ++ = (CUnsignedIntType)(*p_src ++);
					m_n_string_buffer_used -= n_length;
					memcpy(m_p_string_buffer, m_p_string_buffer + n_length,
						m_n_string_buffer_used);
				}
				// output the whole sequence to the string buffer, use part, keep the rest
				return true;
			}
		} else if(m_t_overflow_proc.n_mode == wchar_Replace) {
			*p_dest ++ = m_t_overflow_proc.n_replacement_char;
			return true;
		} else if(m_t_overflow_proc.n_mode == wchar_Discard)
			return true;
		else /*if(m_t_overflow_proc.n_mode == wchar_Fail)*/ {
			_ASSERTE(m_t_overflow_proc.n_mode == wchar_Fail);
			Raise(error_Threshold);
			return false;
		}
	}
#endif // __UNICODE_FILE_ENABLE_CHAR_PROCESSING
};

#endif // !__UNICODE_INCLUDED
