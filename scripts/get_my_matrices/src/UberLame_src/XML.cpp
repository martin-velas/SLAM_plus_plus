/*
								+----------------------------------+
								|                                  |
								|    ***  Tiny XML parser   ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|             XML.cpp              |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file XML.cpp
 *	@date 2008
 *	@author -tHE SWINe-
 *	@brief tiny XML parser
 *
 *	@date 2008-04-21
 *
 *	first version with event-based xml parser
 *
 *	parser is able to read XML 1.0, but it doesn't support document type definitions (it can
 *	parse <!DOCUMENT element with link to external DTD, but it doesn't parse internal subsets, or
 *	referenced external DTD). this feature is most likely not going to be implemented.
 *
 *	the abilities can be compared to the TinyXML library which does in addition build DOM which can
 *	be edited and written back to a XML file. on the other side TinyXML doesn't support utf-16 nor
 *	iso-8859-n encodings (at least today) and is notably slower (TinyXML runs at 65% speed of this
 *	parser, tested by reading 100MB xml file several times), but still - this parser doesn't build DOM.
 *
 *	t_odo - debug this, see how about xml1.0 compliance
 *	todo - write DOM parser and document writer
 *
 *	@date 2008-05-06
 *
 *	fixed typo CXML_EventParser::mode_DocTypeDecl (should've been node_DocTypeDecl)
 *	added CXML_FlatItemList_Parser
 *
 *	@date 2008-07-02
 *
 *	added two versions of strconv_w() utility function for wide to ascii string conversion
 *
 *	@date 2008-11-21
 *
 *	added six more versions of strconv_w() utility function for wide to ascii string conversion
 *
 *	@date 2008-11-25
 *
 *	added CWideUtils class
 *
 *	t_odo - add unicode conversion options to CXML_Lexer (raw unicode (current), force 8-bit,
 *	utf-16, utf-8) so all strings, coming out of CXML_Lexer are in proper format. this should
 *	yield much user-friendlier interface (simple std::strings, no strconv_w calls, etc.)
 *
 *	@date 2008-12-13
 *
 *	implemented transparent unicode conversion via CXML_Lexer::AppendChar(). desired charset
 *	is set by defining one of __XML_LEXER_FORMAT_UTF_32, __XML_LEXER_FORMAT_RAW_16,
 *	__XML_LEXER_FORMAT_RAW_8, __XML_LEXER_FORMAT_UTF_16 or __XML_LEXER_FORMAT_UTF_8
 *
 *	t_odo - parse DOM
 *
 *	@date 2008-12-21
 *
 *	added CXMLNode class, representing parse-tree node, similar to one of Tiny XML (but much
 *	simpler and read only). still attaining speed ratio arround 2:1 (ulame:tixml).
 *
 *	tested under g++, removed warnings
 *
 *	@date 2009-01-13
 *
 *	fixed error in CXML_Lexer::ReplaceCharRefs() where numerical character references werent
 *	recognized properly
 *
 *	latest benchmarks shows speeds 13MB/s event-based, 11MB/s CXMLNode and 7MB/s TinyXml
 *
 *	@date 2009-02-13
 *
 *	added CXMLNode::p_ToElement(), CXMLNode::p_ToText(), CXMLNode::p_ToCData() and
 *	CXMLNode::s_InnerText() convenience functions
 *
 *	@date 2009-05-23
 *
 *	removed all instances of std::vector::reserve and replaced them by stl_ut::Reserve_*
 *
 *	@date 2009-10-11
 *
 *	replaced stl container ::resize() by stl_ut::Resize_*() to avoid unhandled
 *	std::bad_alloc
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 */

#include "NewFix.h"

#include "CallStack.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>
#include "Scanner.h"
#include "Unicode.h"
#include "XML.h"
#include "StlUtils.h"
#if defined(__XML_LEXER_FORMAT_UTF_16) || defined(__XML_LEXER_FORMAT_UTF_8)
#include "UniConv.h"
#endif // __XML_LEXER_FORMAT_UTF_16 || __XML_LEXER_FORMAT_UTF_8

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/*
 *								=== CWideUtils ===
 */

size_t CWideUtils::strlen_w(const int *p_s_buffer)
{
	const int *p_s_start = p_s_buffer;
	while(*p_s_buffer)
		++ p_s_buffer;
	return p_s_buffer - p_s_start;
}

#ifdef __DEPRECATED_WIDE_UTILS

char *CWideUtils::strconv_w(const CXML_Lexer::wstring &r_s_string)
{
	char *p_s_buffer;
	if(!(p_s_buffer = new(std::nothrow) char[r_s_string.length() + 1]))
		return 0;
	char *p_s_dest = p_s_buffer;
	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i, ++ p_s_dest) {
		int n_char = r_s_string[i];
		if(n_char < 0 || n_char > 255) {
			delete[] p_s_buffer;
			return 0; // unconvertible char
		}
		*p_s_dest = (char)n_char;
	}
	*p_s_dest = 0; // terminating 0
	return p_s_buffer;
}

char *CWideUtils::strconv_w(const int *p_s_buffer)
{
	char *p_s_narrow_buffer;
	if(!(p_s_narrow_buffer = new(std::nothrow) char[strlen_w(p_s_buffer) + 1]))
		return 0;
	char *p_s_dest = p_s_narrow_buffer;
	for(; *p_s_buffer; ++ p_s_buffer, ++ p_s_dest) {
		int n_char = *p_s_buffer;
		if(n_char < 0 || n_char > 255) {
			delete[] p_s_narrow_buffer;
			return 0; // unconvertible char
		}
		*p_s_dest = (char)n_char;
	}
	*p_s_dest = 0; // terminating 0
	return p_s_narrow_buffer;
}

bool CWideUtils::strconv_w(std::string &r_s_dest, const CXML_Lexer::wstring &r_s_string)
{
	if(!stl_ut::Resize_To_N(r_s_dest, r_s_string.length()))
		return false;
	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i) {
		int n_char = r_s_string[i];
		if(n_char < 0 || n_char > 255)
			return false; // unconvertible char
		r_s_dest[i] = char(n_char);
	}
	return true;
}

bool CWideUtils::strconv_w(std::string &r_s_dest, const int *p_s_buffer)
{
	size_t n_length = strlen_w(p_s_buffer);
	if(!stl_ut::Resize_To_N(r_s_dest, n_length))
		return false;
	for(size_t i = 0, n = n_length; i < n; ++ i) {
		int n_char = *p_s_buffer ++;
		if(n_char < 0 || n_char > 255)
			return false; // unconvertible char
		r_s_dest[i] = char(n_char);
	}
	return true;
}

char *CWideUtils::strconv_w(const CXML_Lexer::wstring &r_s_string, char n_replacement)
{
	char *p_s_buffer;
	if(!(p_s_buffer = new(std::nothrow) char[r_s_string.length() + 1]))
		return 0;
	char *p_s_dest = p_s_buffer;
	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i, ++ p_s_dest) {
		int n_char = r_s_string[i];
		if(n_char < 0 || n_char > 255)
			*p_s_dest = n_replacement; // unconvertible char
		else
			*p_s_dest = (char)n_char;
	}
	*p_s_dest = 0; // terminating 0
	return p_s_buffer;
}

char *CWideUtils::strconv_w(const int *p_s_buffer, char n_replacement)
{
	char *p_s_narrow_buffer;
	if(!(p_s_narrow_buffer = new(std::nothrow) char[strlen_w(p_s_buffer) + 1]))
		return 0;
	char *p_s_dest = p_s_narrow_buffer;
	for(; *p_s_buffer; ++ p_s_buffer, ++ p_s_dest) {
		int n_char = *p_s_buffer;
		if(n_char < 0 || n_char > 255)
			*p_s_dest = n_replacement; // unconvertible char
		else
			*p_s_dest = (char)n_char;
	}
	*p_s_dest = 0; // terminating 0
	return p_s_narrow_buffer;
}

bool CWideUtils::strconv_w(std::string &r_s_dest,
	const CXML_Lexer::wstring &r_s_string, char n_replacement)
{
	if(!stl_ut::Resize_To_N(r_s_dest, r_s_string.length()))
		return false;
	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i) {
		int n_char = r_s_string[i];
		if(n_char < 0 || n_char > 255)
			r_s_dest[i] = n_replacement; // unconvertible char
		else
			r_s_dest[i] = char(n_char);
	}
	return true;
}

bool CWideUtils::strconv_w(std::string &r_s_dest,
	const int *p_s_buffer, char n_replacement)
{
	size_t n_length = strlen_w(p_s_buffer);
	if(!stl_ut::Resize_To_N(r_s_dest, n_length))
		return false;
	for(size_t i = 0, n = n_length; i < n; ++ i) {
		int n_char = *p_s_buffer ++;
		if(n_char < 0 || n_char > 255)
			r_s_dest[i] = n_replacement;
		else
			r_s_dest[i] = char(n_char);
	}
	return true;
}

long CWideUtils::atol_w(const int *p_s_buffer)
{
    while(isspace(*p_s_buffer))
        ++ p_s_buffer;
	// skip whitespace

    int sign = *p_s_buffer;
    if(sign == '-' || sign == '+')
        ++ p_s_buffer;
    // skip sign

    long total = 0;
    for(int c; isdigit(c = *p_s_buffer); ++ p_s_buffer)
        total = 10 * total + (c - '0');
	// accumulate digits

    if(sign == '-')
        return -total;
    else
        return total;
	// return result, negated if necessary
}

int CWideUtils::atoi_w(const int *p_s_buffer)
{
    while(isspace(*p_s_buffer))
        ++ p_s_buffer;
	// skip whitespace

    int sign = *p_s_buffer;
    if(sign == '-' || sign == '+')
        ++ p_s_buffer;
    // skip sign

    int total = 0;
    for(int c; isdigit(c = *p_s_buffer); ++ p_s_buffer)
        total = 10 * total + (c - '0');
	// accumulate digits

    if(sign == '-')
        return -total;
    else
        return total;
	// return result, negated if necessary
}

long CWideUtils::atol_w(const CXML_Lexer::wstring &r_s_string)
{
	size_t b = 0, e = r_s_string.length();
    while(b < e && isspace(r_s_string[b]))
        ++ b;
	// skip whitespace

    int sign = r_s_string[b];
    if(sign == '-' || sign == '+')
        ++ b;
    // skip sign

    long total = 0;
    for(int c; b < e && isdigit(c = r_s_string[b]); ++ b)
        total = 10 * total + (c - '0');
	// accumulate digits

    if(sign == '-')
        return -total;
    else
        return total;
	// return result, negated if necessary
}

int CWideUtils::atoi_w(const CXML_Lexer::wstring &r_s_string)
{
	size_t b = 0, e = r_s_string.length();
    while(b < e && isspace(r_s_string[b]))
        ++ b;
	// skip whitespace

    int sign = r_s_string[b];
    if(sign == '-' || sign == '+')
        ++ b;
    // skip sign

    int total = 0;
    for(int c; b < e && isdigit(c = r_s_string[b]); ++ b)
        total = 10 * total + (c - '0');
	// accumulate digits

    if(sign == '-')
        return -total;
    else
        return total;
	// return result, negated if necessary
}

bool CWideUtils::strimatch_w(const int *p_s_buffer, const char *p_s_match)
{
	int a, b;
	while((a = *p_s_buffer) && (b = *p_s_match)) {
		_ASSERTE(tolower(uint8_t(b)) == b);
		if((a < 0x80 && tolower(a) != b) || (a >= 0x80 && a != b))
			return false;
	}
	return (a >= 0x80 || tolower(a) == b) && (a < 0x80 || a == b);
}

bool CWideUtils::strinmatch_w(const int *p_s_buffer, const char *p_s_match, size_t n_length)
{
	int a, b;
	for(; n_length && (a = *p_s_buffer) && (b = *p_s_match); -- n_length) {
		_ASSERTE(tolower(uint8_t(b)) == b);
		if((a < 0x80 && tolower(a) != b) || (a >= 0x80 && a != b))
			return false;
	}
	return true;
}

bool CWideUtils::strimatch_w(const CXML_Lexer::wstring &r_s_string, const char *p_s_name)
{
	size_t n_length;
	if((n_length = r_s_string.length()) != strlen(p_s_name))
		return false;
	for(size_t i = 0; i < n_length; ++ i, ++ p_s_name) {
		_ASSERTE(tolower(uint8_t(*p_s_name)) == *p_s_name);
		if((r_s_string[i] < 0x80 && tolower(r_s_string[i]) != *p_s_name) ||
		   (r_s_string[i] >= 0x80 && r_s_string[i] != *p_s_name))
			return false;
	}
	return true;
}

#endif // __DEPRECATED_WIDE_UTILS

bool CWideUtils::strmatch_w(const CXML_Lexer::wstring &r_s_string, const char *p_s_name)
{
	size_t n_length;
	if((n_length = r_s_string.length()) != strlen(p_s_name))
		return false;
	for(size_t i = 0; i < n_length; ++ i, ++ p_s_name) {
		if(r_s_string[i] != *p_s_name)
			return false;
	}
	return true;
}

/*
 *								=== ~CWideUtils ===
 */

/*
 *								=== CXML_Lexer::TToken ===
 */

/*
 *	inline CXML_Lexer::TToken::TToken(int _n_line, int _n_col, int _n_type)
 *		- creates a new token, starting on line _n_line and column _n_col
 *		- token type is _n_type
 */
inline CXML_Lexer::TToken::TToken(int _n_line, int _n_col, int _n_type)
	:n_line(_n_line), n_col(_n_col), n_type(_n_type)
{
	__FuncGuard("CXML_Lexer::TToken::TToken");
}

/*
 *	CXML_Lexer::TToken::TToken(int _n_line, int _n_col, int _n_type, const int *p_s_buffer)
 *		- creates a new token, starting on line _n_line and column _n_col
 *		- token type is _n_type and p_s_buffer points to unicode string with token contents
 *		- if there is not enough memory to create copy of the string,
 *		  token type is set to error_Memory. if code of characters in p_s_buffer
 *		  is greater than TCharType can contain, token type is set to error_Code
 */
CXML_Lexer::TToken::TToken(int _n_line, int _n_col, int _n_type, const int *p_s_buffer)
	:n_line(_n_line), n_col(_n_col), n_type(_n_type)
{
	__FuncGuard("CXML_Lexer::TToken::TToken");

	for(; *p_s_buffer; ++ p_s_buffer) {
		int n_result;
		if((n_result = AppendChar(s_value, *p_s_buffer))) {
			n_type = n_result;
			return;
		}
	}
	// alloc and copy string
}

/*
 *	CXML_Lexer::TToken::TToken(int _n_line, int _n_col,
 *		int _n_type, const int *p_s_buffer, int n_length)
 *		- creates a new token, starting on line _n_line and column _n_col
 *		- token type is _n_type, p_s_buffer points to unicode string with
 *		  token contents and n_length contains length of string in p_s_buffer
 *		- if there is not enough memory to create copy of the string,
 *		  token type is set to error_Memory. if code of characters in p_s_buffer
 *		  is greater than TCharType can contain, token type is set to error_Code
 */
CXML_Lexer::TToken::TToken(int _n_line, int _n_col, int _n_type,
	const int *p_s_buffer, size_t n_length)
	:n_line(_n_line), n_col(_n_col), n_type(_n_type)
{
	__FuncGuard("CXML_Lexer::TToken::TToken");

	for(; n_length > 0; ++ p_s_buffer, -- n_length) {
		int n_result;
		if((n_result = AppendChar(s_value, *p_s_buffer))) {
			n_type = n_result;
			return;
		}
	}
	// alloc and copy string
}

#ifdef __XML_LEXER_TOKEN_DUMP

/*
 *	void CXML_Lexer::TToken::Dump() const
 *		- prints token contents to stdout
 *		- note this is compiled only if __XML_LEXER_TOKEN_DUMP is defined
 */
void CXML_Lexer::TToken::Dump() const
{
	__FuncGuard("CXML_Lexer::TToken::Dump");

	const char *p_type_name_list[] = {
		"e_m", "e_c",
		"token_White",
		"token_Comment",
		"token_CData",
		"token_Elem(<)",
		"token_EndElem(</)",
		"token_PIElem(<?)",
		"token_DTDElem(<!)",
		"token_End(>)",
		"token_SingleEnd(/>)",
		"token_PIEnd(?>)",
		"token_Assign",
		"token_Name",
		"token_AttValue",
		"token_EOF"
	};

	if(n_type >= token_Elem && n_type <= token_DTDElem) {
		printf("TToken(n_line: %d, n_col: %d, n_type: %s)\n",
			n_line, n_col, p_type_name_list[n_type + 2]);
	} else if(n_type == token_Name) {
		printf("TToken(n_line: %d, n_col: %d, n_type: %s, s_name: %s)\n",
			n_line, n_col, p_type_name_list[n_type + 2], s_value.c_str());
	} else if(n_type == token_CData || n_type == token_AttValue) {
		printf("TToken(n_line: %d, n_col: %d, n_type: %s, s_value: \'%s\')\n",
			n_line, n_col, p_type_name_list[n_type + 2], s_value.c_str());
	} else {
		printf("TToken(n_line: %d, n_col: %d, n_type: %s)\n",
			n_line, n_col, p_type_name_list[n_type + 2]);
	}
}

#endif // __XML_LEXER_TOKEN_DUMP

/*
 *								=== ~CXML_Lexer::TToken ===
 */

/*
 *								=== CXML_Lexer::CFileReader ===
 */

CXML_Lexer::CFileReader::CFileReader(const char *p_s_filename)
	:m_n_cached_char(0), m_n_io_error(CUnicodeFile::error_NoError)
{
	__FuncGuard("CXML_Lexer::CFileReader::CFileReader");

	if(!m_file.Open_Read(p_s_filename, true, CUnicodeFile::code_UTF_8)) {
		m_n_io_error = CUnicodeFile::error_IO;
		return;
	}
}

CXML_Lexer::CFileReader::~CFileReader()
{
	__FuncGuard("CXML_Lexer::CFileReader::~CFileReader");

	if(m_file.b_Opened())
		m_file.Close(); // propably would do it itself
}

bool CXML_Lexer::CFileReader::SetEncoding(int n_encoding) // forces encoding
{
	__FuncGuard("CXML_Lexer::CFileReader::SetEncoding");

	if(m_file.b_Contains_BOM() && m_file.n_Encoding() == n_encoding)
		return true;
	return m_file.SetEncoding(n_encoding);
}

bool CXML_Lexer::CFileReader::b_Status() const
{
	return m_file.b_Opened();
}

int CXML_Lexer::CFileReader::n_IO_Error() const
{
	return m_n_io_error;
}

inline bool CXML_Lexer::CFileReader::ReadChar(int &r_n_char)
{
	__FuncGuard("CXML_Lexer::CFileReader::ReadChar");

	if(m_n_cached_char) {
		r_n_char = m_n_cached_char;
		m_n_cached_char = 0;
		return true;
	}
	if((r_n_char = m_file.n_GetChar()) > 0)
		return true;
	m_n_io_error = (m_file.n_GetError() != CUnicodeFile::error_NoError);
	return false;
	// on EOF returns -1 but raises no error
}

inline bool CXML_Lexer::CFileReader::UnreadChar(int n_char)
{
	__FuncGuard("CXML_Lexer::CFileReader::UnreadChar");

	_ASSERTE(!m_n_cached_char);
	return (m_n_cached_char = n_char) != 0;
}

/*
 *								=== ~CXML_Lexer::CFileReader ===
 */

/*
 *								=== CXML_Lexer::CTokenEmitAdapter ===
 */

inline CXML_Lexer::CTokenEmitAdapter::CTokenEmitAdapter(TToken &r_t_out_token,
	TokenEmitFunc *p_emit_func)
	:m_p_out_token(&r_t_out_token), m_p_emit_func(p_emit_func)
{
	__FuncGuard("CXML_Lexer::CTokenEmitAdapter::CTokenEmitAdapter");
}

inline bool CXML_Lexer::CTokenEmitAdapter::operator ()(const int *p_s_buffer,
	int n_regexp_id, int n_line, int n_column) const
{
	__FuncGuard("CXML_Lexer::CTokenEmitAdapter::operator ()");

	return (m_p_emit_func[n_regexp_id])(p_s_buffer, n_regexp_id, n_line, n_column, m_p_out_token);
	// call ordinary function
}

/*
 *								=== ~CXML_Lexer::CTokenEmitAdapter ===
 */

/*
 *								=== CXML_Lexer ===
 */

//const int CXML_Lexer::n_Char_Max = maxvalueof(TCharType);

#ifdef __XML_LEXER_XML10_COMPLIANT_CHARSET
//#include "XML_LexerTables_Full.h" // m_p_transition, m_p_state

const CXML_Lexer::TTransition CXML_Lexer::m_p_transition[1654] = {
	{0x3d, 0x3d, 7}, {0x3c, 0x3c, 6}, {0x2f, 0x2f, 4}, {0x27, 0x27, 3}, {0xa, 0x9, 1},
	{0xd, 0xd, 1}, {0x20, 0x20, 1}, {0x5a, 0x41, 5}, {0x7a, 0x61, 5}, {0xd6, 0xc0, 5},
	{0xf6, 0xd8, 5}, {0x131, 0xf8, 5}, {0x13e, 0x134, 5}, {0x148, 0x141, 5}, {0x17e, 0x14a, 5},
	{0x1c3, 0x180, 5}, {0x1f0, 0x1cd, 5}, {0x1f5, 0x1f4, 5}, {0x217, 0x1fa, 5}, {0x2a8, 0x250, 5},
	{0x2c1, 0x2bb, 5}, {0x386, 0x386, 5}, {0x38a, 0x388, 5}, {0x38c, 0x38c, 5}, {0x3a1, 0x38e, 5},
	{0x3ce, 0x3a3, 5}, {0x3d6, 0x3d0, 5}, {0x3da, 0x3da, 5}, {0x3dc, 0x3dc, 5}, {0x3de, 0x3de, 5},
	{0x3e0, 0x3e0, 5}, {0x3f3, 0x3e2, 5}, {0x40c, 0x401, 5}, {0x44f, 0x40e, 5}, {0x45c, 0x451, 5},
	{0x481, 0x45e, 5}, {0x4c4, 0x490, 5}, {0x4c8, 0x4c7, 5}, {0x4cc, 0x4cb, 5}, {0x4eb, 0x4d0, 5},
	{0x4f5, 0x4ee, 5}, {0x4f9, 0x4f8, 5}, {0x556, 0x531, 5}, {0x559, 0x559, 5}, {0x586, 0x561, 5},
	{0x5ea, 0x5d0, 5}, {0x5f2, 0x5f0, 5}, {0x63a, 0x621, 5}, {0x64a, 0x641, 5}, {0x6b7, 0x671, 5},
	{0x6be, 0x6ba, 5}, {0x6ce, 0x6c0, 5}, {0x6d3, 0x6d0, 5}, {0x6d5, 0x6d5, 5}, {0x6e6, 0x6e5, 5},
	{0x939, 0x905, 5}, {0x93d, 0x93d, 5}, {0x961, 0x958, 5}, {0x98c, 0x985, 5}, {0x990, 0x98f, 5},
	{0x9a8, 0x993, 5}, {0x9b0, 0x9aa, 5}, {0x9b2, 0x9b2, 5}, {0x9b9, 0x9b6, 5}, {0x9dd, 0x9dc, 5},
	{0x9e1, 0x9df, 5}, {0x9f1, 0x9f0, 5}, {0xa0a, 0xa05, 5}, {0xa10, 0xa0f, 5}, {0xa28, 0xa13, 5},
	{0xa30, 0xa2a, 5}, {0xa33, 0xa32, 5}, {0xa36, 0xa35, 5}, {0xa39, 0xa38, 5}, {0xa5c, 0xa59, 5},
	{0xa5e, 0xa5e, 5}, {0xa74, 0xa72, 5}, {0xa8b, 0xa85, 5}, {0xa8d, 0xa8d, 5}, {0xa91, 0xa8f, 5},
	{0xaa8, 0xa93, 5}, {0xab0, 0xaaa, 5}, {0xab3, 0xab2, 5}, {0xab9, 0xab5, 5}, {0xabd, 0xabd, 5},
	{0xae0, 0xae0, 5}, {0xb0c, 0xb05, 5}, {0xb10, 0xb0f, 5}, {0xb28, 0xb13, 5}, {0xb30, 0xb2a, 5},
	{0xb33, 0xb32, 5}, {0xb39, 0xb36, 5}, {0xb3d, 0xb3d, 5}, {0xb5d, 0xb5c, 5}, {0xb61, 0xb5f, 5},
	{0xb8a, 0xb85, 5}, {0xb90, 0xb8e, 5}, {0xb95, 0xb92, 5}, {0xb9a, 0xb99, 5}, {0xb9c, 0xb9c, 5},
	{0xb9f, 0xb9e, 5}, {0xba4, 0xba3, 5}, {0xbaa, 0xba8, 5}, {0xbb5, 0xbae, 5}, {0xbb9, 0xbb7, 5},
	{0xc0c, 0xc05, 5}, {0xc10, 0xc0e, 5}, {0xc28, 0xc12, 5}, {0xc33, 0xc2a, 5}, {0xc39, 0xc35, 5},
	{0xc61, 0xc60, 5}, {0xc8c, 0xc85, 5}, {0xc90, 0xc8e, 5}, {0xca8, 0xc92, 5}, {0xcb3, 0xcaa, 5},
	{0xcb9, 0xcb5, 5}, {0xcde, 0xcde, 5}, {0xce1, 0xce0, 5}, {0xd0c, 0xd05, 5}, {0xd10, 0xd0e, 5},
	{0xd28, 0xd12, 5}, {0xd39, 0xd2a, 5}, {0xd61, 0xd60, 5}, {0xe2e, 0xe01, 5}, {0xe30, 0xe30, 5},
	{0xe33, 0xe32, 5}, {0xe45, 0xe40, 5}, {0xe82, 0xe81, 5}, {0xe84, 0xe84, 5}, {0xe88, 0xe87, 5},
	{0xe8a, 0xe8a, 5}, {0xe8d, 0xe8d, 5}, {0xe97, 0xe94, 5}, {0xe9f, 0xe99, 5}, {0xea3, 0xea1, 5},
	{0xea5, 0xea5, 5}, {0xea7, 0xea7, 5}, {0xeab, 0xeaa, 5}, {0xeae, 0xead, 5}, {0xeb0, 0xeb0, 5},
	{0xeb3, 0xeb2, 5}, {0xebd, 0xebd, 5}, {0xec4, 0xec0, 5}, {0xf47, 0xf40, 5}, {0xf69, 0xf49, 5},
	{0x10c5, 0x10a0, 5}, {0x10f6, 0x10d0, 5}, {0x1100, 0x1100, 5}, {0x1103, 0x1102, 5},
	{0x1107, 0x1105, 5}, {0x1109, 0x1109, 5}, {0x110c, 0x110b, 5}, {0x1112, 0x110e, 5},
	{0x113c, 0x113c, 5}, {0x113e, 0x113e, 5}, {0x1140, 0x1140, 5}, {0x114c, 0x114c, 5},
	{0x114e, 0x114e, 5}, {0x1150, 0x1150, 5}, {0x1155, 0x1154, 5}, {0x1159, 0x1159, 5},
	{0x1161, 0x115f, 5}, {0x1163, 0x1163, 5}, {0x1165, 0x1165, 5}, {0x1167, 0x1167, 5},
	{0x1169, 0x1169, 5}, {0x116e, 0x116d, 5}, {0x1173, 0x1172, 5}, {0x1175, 0x1175, 5},
	{0x119e, 0x119e, 5}, {0x11a8, 0x11a8, 5}, {0x11ab, 0x11ab, 5}, {0x11af, 0x11ae, 5},
	{0x11b8, 0x11b7, 5}, {0x11ba, 0x11ba, 5}, {0x11c2, 0x11bc, 5}, {0x11eb, 0x11eb, 5},
	{0x11f0, 0x11f0, 5}, {0x11f9, 0x11f9, 5}, {0x1e9b, 0x1e00, 5}, {0x1ef9, 0x1ea0, 5},
	{0x1f15, 0x1f00, 5}, {0x1f1d, 0x1f18, 5}, {0x1f45, 0x1f20, 5}, {0x1f4d, 0x1f48, 5},
	{0x1f57, 0x1f50, 5}, {0x1f59, 0x1f59, 5}, {0x1f5b, 0x1f5b, 5}, {0x1f5d, 0x1f5d, 5},
	{0x1f7d, 0x1f5f, 5}, {0x1fb4, 0x1f80, 5}, {0x1fbc, 0x1fb6, 5}, {0x1fbe, 0x1fbe, 5},
	{0x1fc4, 0x1fc2, 5}, {0x1fcc, 0x1fc6, 5}, {0x1fd3, 0x1fd0, 5}, {0x1fdb, 0x1fd6, 5},
	{0x1fec, 0x1fe0, 5}, {0x1ff4, 0x1ff2, 5}, {0x1ffc, 0x1ff6, 5}, {0x2126, 0x2126, 5},
	{0x212b, 0x212a, 5}, {0x212e, 0x212e, 5}, {0x2182, 0x2180, 5}, {0x3094, 0x3041, 5},
	{0x30fa, 0x30a1, 5}, {0x312c, 0x3105, 5}, {0xd7a3, 0xac00, 5}, {0x3007, 0x3007, 5},
	{0x3029, 0x3021, 5}, {0x9fa5, 0x4e00, 5}, {0x3f, 0x3f, 9}, {0x3e, 0x3e, 8}, {0x3a, 0x3a, 5},
	{0x5f, 0x5f, 5}, {0x22, 0x22, 2}, {0xa, 0x9, 1}, {0xd, 0xd, 1}, {0x20, 0x20, 1},
	{0x22, 0x22, 32}, {0x26, 0x26, 38}, {0x21, 0x0, 2}, {0x25, 0x23, 2}, {0x3b, 0x27, 2},
	{0x10ffff, 0x3d, 2}, {0x25, 0x0, 3}, {0x3b, 0x28, 3}, {0x10ffff, 0x3d, 3}, {0x27, 0x27, 32},
	{0x26, 0x26, 31}, {0x3e, 0x3e, 30}, {0x5a, 0x41, 5}, {0x7a, 0x61, 5}, {0xd6, 0xc0, 5},
	{0xf6, 0xd8, 5}, {0x131, 0xf8, 5}, {0x13e, 0x134, 5}, {0x148, 0x141, 5}, {0x17e, 0x14a, 5},
	{0x1c3, 0x180, 5}, {0x1f0, 0x1cd, 5}, {0x1f5, 0x1f4, 5}, {0x217, 0x1fa, 5}, {0x2a8, 0x250, 5},
	{0x2c1, 0x2bb, 5}, {0x386, 0x386, 5}, {0x38a, 0x388, 5}, {0x38c, 0x38c, 5}, {0x3a1, 0x38e, 5},
	{0x3ce, 0x3a3, 5}, {0x3d6, 0x3d0, 5}, {0x3da, 0x3da, 5}, {0x3dc, 0x3dc, 5}, {0x3de, 0x3de, 5},
	{0x3e0, 0x3e0, 5}, {0x3f3, 0x3e2, 5}, {0x40c, 0x401, 5}, {0x44f, 0x40e, 5}, {0x45c, 0x451, 5},
	{0x481, 0x45e, 5}, {0x4c4, 0x490, 5}, {0x4c8, 0x4c7, 5}, {0x4cc, 0x4cb, 5}, {0x4eb, 0x4d0, 5},
	{0x4f5, 0x4ee, 5}, {0x4f9, 0x4f8, 5}, {0x556, 0x531, 5}, {0x559, 0x559, 5}, {0x586, 0x561, 5},
	{0x5ea, 0x5d0, 5}, {0x5f2, 0x5f0, 5}, {0x63a, 0x621, 5}, {0x64a, 0x641, 5}, {0x6b7, 0x671, 5},
	{0x6be, 0x6ba, 5}, {0x6ce, 0x6c0, 5}, {0x6d3, 0x6d0, 5}, {0x6d5, 0x6d5, 5}, {0x6e6, 0x6e5, 5},
	{0x939, 0x905, 5}, {0x93d, 0x93d, 5}, {0x961, 0x958, 5}, {0x98c, 0x985, 5}, {0x990, 0x98f, 5},
	{0x9a8, 0x993, 5}, {0x9b0, 0x9aa, 5}, {0x9b2, 0x9b2, 5}, {0x9b9, 0x9b6, 5}, {0x9dd, 0x9dc, 5},
	{0x9e1, 0x9df, 5}, {0x9f1, 0x9f0, 5}, {0xa0a, 0xa05, 5}, {0xa10, 0xa0f, 5}, {0xa28, 0xa13, 5},
	{0xa30, 0xa2a, 5}, {0xa33, 0xa32, 5}, {0xa36, 0xa35, 5}, {0xa39, 0xa38, 5}, {0xa5c, 0xa59, 5},
	{0xa5e, 0xa5e, 5}, {0xa74, 0xa72, 5}, {0xa8b, 0xa85, 5}, {0xa8d, 0xa8d, 5}, {0xa91, 0xa8f, 5},
	{0xaa8, 0xa93, 5}, {0xab0, 0xaaa, 5}, {0xab3, 0xab2, 5}, {0xab9, 0xab5, 5}, {0xabd, 0xabd, 5},
	{0xae0, 0xae0, 5}, {0xb0c, 0xb05, 5}, {0xb10, 0xb0f, 5}, {0xb28, 0xb13, 5}, {0xb30, 0xb2a, 5},
	{0xb33, 0xb32, 5}, {0xb39, 0xb36, 5}, {0xb3d, 0xb3d, 5}, {0xb5d, 0xb5c, 5}, {0xb61, 0xb5f, 5},
	{0xb8a, 0xb85, 5}, {0xb90, 0xb8e, 5}, {0xb95, 0xb92, 5}, {0xb9a, 0xb99, 5}, {0xb9c, 0xb9c, 5},
	{0xb9f, 0xb9e, 5}, {0xba4, 0xba3, 5}, {0xbaa, 0xba8, 5}, {0xbb5, 0xbae, 5}, {0xbb9, 0xbb7, 5},
	{0xc0c, 0xc05, 5}, {0xc10, 0xc0e, 5}, {0xc28, 0xc12, 5}, {0xc33, 0xc2a, 5}, {0xc39, 0xc35, 5},
	{0xc61, 0xc60, 5}, {0xc8c, 0xc85, 5}, {0xc90, 0xc8e, 5}, {0xca8, 0xc92, 5}, {0xcb3, 0xcaa, 5},
	{0xcb9, 0xcb5, 5}, {0xcde, 0xcde, 5}, {0xce1, 0xce0, 5}, {0xd0c, 0xd05, 5}, {0xd10, 0xd0e, 5},
	{0xd28, 0xd12, 5}, {0xd39, 0xd2a, 5}, {0xd61, 0xd60, 5}, {0xe2e, 0xe01, 5}, {0xe30, 0xe30, 5},
	{0xe33, 0xe32, 5}, {0xe45, 0xe40, 5}, {0xe82, 0xe81, 5}, {0xe84, 0xe84, 5}, {0xe88, 0xe87, 5},
	{0xe8a, 0xe8a, 5}, {0xe8d, 0xe8d, 5}, {0xe97, 0xe94, 5}, {0xe9f, 0xe99, 5}, {0xea3, 0xea1, 5},
	{0xea5, 0xea5, 5}, {0xea7, 0xea7, 5}, {0xeab, 0xeaa, 5}, {0xeae, 0xead, 5}, {0xeb0, 0xeb0, 5},
	{0xeb3, 0xeb2, 5}, {0xebd, 0xebd, 5}, {0xec4, 0xec0, 5}, {0xf47, 0xf40, 5}, {0xf69, 0xf49, 5},
	{0x10c5, 0x10a0, 5}, {0x10f6, 0x10d0, 5}, {0x1100, 0x1100, 5}, {0x1103, 0x1102, 5},
	{0x1107, 0x1105, 5}, {0x1109, 0x1109, 5}, {0x110c, 0x110b, 5}, {0x1112, 0x110e, 5},
	{0x113c, 0x113c, 5}, {0x113e, 0x113e, 5}, {0x1140, 0x1140, 5}, {0x114c, 0x114c, 5},
	{0x114e, 0x114e, 5}, {0x1150, 0x1150, 5}, {0x1155, 0x1154, 5}, {0x1159, 0x1159, 5},
	{0x1161, 0x115f, 5}, {0x1163, 0x1163, 5}, {0x1165, 0x1165, 5}, {0x1167, 0x1167, 5},
	{0x1169, 0x1169, 5}, {0x116e, 0x116d, 5}, {0x1173, 0x1172, 5}, {0x1175, 0x1175, 5},
	{0x119e, 0x119e, 5}, {0x11a8, 0x11a8, 5}, {0x11ab, 0x11ab, 5}, {0x11af, 0x11ae, 5},
	{0x11b8, 0x11b7, 5}, {0x11ba, 0x11ba, 5}, {0x11c2, 0x11bc, 5}, {0x11eb, 0x11eb, 5},
	{0x11f0, 0x11f0, 5}, {0x11f9, 0x11f9, 5}, {0x1e9b, 0x1e00, 5}, {0x1ef9, 0x1ea0, 5},
	{0x1f15, 0x1f00, 5}, {0x1f1d, 0x1f18, 5}, {0x1f45, 0x1f20, 5}, {0x1f4d, 0x1f48, 5},
	{0x1f57, 0x1f50, 5}, {0x1f59, 0x1f59, 5}, {0x1f5b, 0x1f5b, 5}, {0x1f5d, 0x1f5d, 5},
	{0x1f7d, 0x1f5f, 5}, {0x1fb4, 0x1f80, 5}, {0x1fbc, 0x1fb6, 5}, {0x1fbe, 0x1fbe, 5},
	{0x1fc4, 0x1fc2, 5}, {0x1fcc, 0x1fc6, 5}, {0x1fd3, 0x1fd0, 5}, {0x1fdb, 0x1fd6, 5},
	{0x1fec, 0x1fe0, 5}, {0x1ff4, 0x1ff2, 5}, {0x1ffc, 0x1ff6, 5}, {0x2126, 0x2126, 5},
	{0x212b, 0x212a, 5}, {0x212e, 0x212e, 5}, {0x2182, 0x2180, 5}, {0x3094, 0x3041, 5},
	{0x30fa, 0x30a1, 5}, {0x312c, 0x3105, 5}, {0xd7a3, 0xac00, 5}, {0x3007, 0x3007, 5},
	{0x3029, 0x3021, 5}, {0x9fa5, 0x4e00, 5}, {0x345, 0x300, 5}, {0x361, 0x360, 5},
	{0x486, 0x483, 5}, {0x5a1, 0x591, 5}, {0x5b9, 0x5a3, 5}, {0x5bd, 0x5bb, 5}, {0x5bf, 0x5bf, 5},
	{0x5c2, 0x5c1, 5}, {0x5c4, 0x5c4, 5}, {0x652, 0x64b, 5}, {0x670, 0x670, 5}, {0x6e4, 0x6d6, 5},
	{0x6e8, 0x6e7, 5}, {0x6ed, 0x6ea, 5}, {0x903, 0x901, 5}, {0x93c, 0x93c, 5}, {0x94d, 0x93e, 5},
	{0x954, 0x951, 5}, {0x963, 0x962, 5}, {0x983, 0x981, 5}, {0x9bc, 0x9bc, 5}, {0x9c4, 0x9be, 5},
	{0x9c8, 0x9c7, 5}, {0x9cd, 0x9cb, 5}, {0x9d7, 0x9d7, 5}, {0x9e3, 0x9e2, 5}, {0xa02, 0xa02, 5},
	{0xa3c, 0xa3c, 5}, {0xa42, 0xa3e, 5}, {0xa48, 0xa47, 5}, {0xa4d, 0xa4b, 5}, {0xa71, 0xa70, 5},
	{0xa83, 0xa81, 5}, {0xabc, 0xabc, 5}, {0xac5, 0xabe, 5}, {0xac9, 0xac7, 5}, {0xacd, 0xacb, 5},
	{0xb03, 0xb01, 5}, {0xb3c, 0xb3c, 5}, {0xb43, 0xb3e, 5}, {0xb48, 0xb47, 5}, {0xb4d, 0xb4b, 5},
	{0xb57, 0xb56, 5}, {0xb83, 0xb82, 5}, {0xbc2, 0xbbe, 5}, {0xbc8, 0xbc6, 5}, {0xbcd, 0xbca, 5},
	{0xbd7, 0xbd7, 5}, {0xc03, 0xc01, 5}, {0xc44, 0xc3e, 5}, {0xc48, 0xc46, 5}, {0xc4d, 0xc4a, 5},
	{0xc56, 0xc55, 5}, {0xc83, 0xc82, 5}, {0xcc4, 0xcbe, 5}, {0xcc8, 0xcc6, 5}, {0xccd, 0xcca, 5},
	{0xcd6, 0xcd5, 5}, {0xd03, 0xd02, 5}, {0xd43, 0xd3e, 5}, {0xd48, 0xd46, 5}, {0xd4d, 0xd4a, 5},
	{0xd57, 0xd57, 5}, {0xe31, 0xe31, 5}, {0xe3a, 0xe34, 5}, {0xe4e, 0xe47, 5}, {0xeb1, 0xeb1, 5},
	{0xeb9, 0xeb4, 5}, {0xebc, 0xebb, 5}, {0xecd, 0xec8, 5}, {0xf19, 0xf18, 5}, {0xf35, 0xf35, 5},
	{0xf37, 0xf37, 5}, {0xf39, 0xf39, 5}, {0xf3f, 0xf3e, 5}, {0xf84, 0xf71, 5}, {0xf8b, 0xf86, 5},
	{0xf95, 0xf90, 5}, {0xf97, 0xf97, 5}, {0xfad, 0xf99, 5}, {0xfb7, 0xfb1, 5}, {0xfb9, 0xfb9, 5},
	{0x20dc, 0x20d0, 5}, {0x20e1, 0x20e1, 5}, {0x302f, 0x302a, 5}, {0x309a, 0x3099, 5},
	{0xb7, 0xb7, 5}, {0x2d1, 0x2d0, 5}, {0x387, 0x387, 5}, {0x640, 0x640, 5}, {0xe46, 0xe46, 5},
	{0xec6, 0xec6, 5}, {0x3005, 0x3005, 5}, {0x3035, 0x3031, 5}, {0x309e, 0x309d, 5},
	{0x30fe, 0x30fc, 5}, {0x39, 0x30, 5}, {0x669, 0x660, 5}, {0x6f9, 0x6f0, 5}, {0x96f, 0x966, 5},
	{0x9ef, 0x9e6, 5}, {0xa6f, 0xa66, 5}, {0xaef, 0xae6, 5}, {0xb6f, 0xb66, 5}, {0xbef, 0xbe7, 5},
	{0xc6f, 0xc66, 5}, {0xcef, 0xce6, 5}, {0xd6f, 0xd66, 5}, {0xe59, 0xe50, 5}, {0xed9, 0xed0, 5},
	{0xf29, 0xf20, 5}, {0x2e, 0x2d, 5}, {0x3a, 0x3a, 5}, {0x5f, 0x5f, 5}, {0x21, 0x21, 11},
	{0x2f, 0x2f, 12}, {0x3f, 0x3f, 13}, {0x3e, 0x3e, 10}, {0x5b, 0x5b, 15}, {0x2d, 0x2d, 14},
	{0x2d, 0x2d, 25}, {0x43, 0x43, 16}, {0x44, 0x44, 17}, {0x41, 0x41, 18}, {0x54, 0x54, 19},
	{0x41, 0x41, 20}, {0x5b, 0x5b, 21}, {0x5d, 0x5d, 22}, {0x5c, 0x0, 21}, {0x10ffff, 0x5e, 21},
	{0x5d, 0x5d, 23}, {0x5c, 0x0, 21}, {0x10ffff, 0x5e, 21}, {0x3d, 0x0, 21}, {0x10ffff, 0x3f, 21},
	{0x3e, 0x3e, 24}, {0x2c, 0x0, 26}, {0x10ffff, 0x2e, 26}, {0x2d, 0x2d, 27}, {0x2c, 0x0, 26},
	{0x10ffff, 0x2e, 26}, {0x2c, 0x0, 26}, {0x10ffff, 0x2e, 26}, {0x2d, 0x2d, 28},
	{0x3e, 0x3e, 29}, {0x3a, 0x3a, 34}, {0x5f, 0x5f, 34}, {0x5a, 0x41, 34}, {0x7a, 0x61, 34},
	{0xd6, 0xc0, 34}, {0xf6, 0xd8, 34}, {0x131, 0xf8, 34}, {0x13e, 0x134, 34}, {0x148, 0x141, 34},
	{0x17e, 0x14a, 34}, {0x1c3, 0x180, 34}, {0x1f0, 0x1cd, 34}, {0x1f5, 0x1f4, 34},
	{0x217, 0x1fa, 34}, {0x2a8, 0x250, 34}, {0x2c1, 0x2bb, 34}, {0x386, 0x386, 34},
	{0x38a, 0x388, 34}, {0x38c, 0x38c, 34}, {0x3a1, 0x38e, 34}, {0x3ce, 0x3a3, 34},
	{0x3d6, 0x3d0, 34}, {0x3da, 0x3da, 34}, {0x3dc, 0x3dc, 34}, {0x3de, 0x3de, 34},
	{0x3e0, 0x3e0, 34}, {0x3f3, 0x3e2, 34}, {0x40c, 0x401, 34}, {0x44f, 0x40e, 34},
	{0x45c, 0x451, 34}, {0x481, 0x45e, 34}, {0x4c4, 0x490, 34}, {0x4c8, 0x4c7, 34},
	{0x4cc, 0x4cb, 34}, {0x4eb, 0x4d0, 34}, {0x4f5, 0x4ee, 34}, {0x4f9, 0x4f8, 34},
	{0x556, 0x531, 34}, {0x559, 0x559, 34}, {0x586, 0x561, 34}, {0x5ea, 0x5d0, 34},
	{0x5f2, 0x5f0, 34}, {0x63a, 0x621, 34}, {0x64a, 0x641, 34}, {0x6b7, 0x671, 34},
	{0x6be, 0x6ba, 34}, {0x6ce, 0x6c0, 34}, {0x6d3, 0x6d0, 34}, {0x6d5, 0x6d5, 34},
	{0x6e6, 0x6e5, 34}, {0x939, 0x905, 34}, {0x93d, 0x93d, 34}, {0x961, 0x958, 34},
	{0x98c, 0x985, 34}, {0x990, 0x98f, 34}, {0x9a8, 0x993, 34}, {0x9b0, 0x9aa, 34},
	{0x9b2, 0x9b2, 34}, {0x9b9, 0x9b6, 34}, {0x9dd, 0x9dc, 34}, {0x9e1, 0x9df, 34},
	{0x9f1, 0x9f0, 34}, {0xa0a, 0xa05, 34}, {0xa10, 0xa0f, 34}, {0xa28, 0xa13, 34},
	{0xa30, 0xa2a, 34}, {0xa33, 0xa32, 34}, {0xa36, 0xa35, 34}, {0xa39, 0xa38, 34},
	{0xa5c, 0xa59, 34}, {0xa5e, 0xa5e, 34}, {0xa74, 0xa72, 34}, {0xa8b, 0xa85, 34},
	{0xa8d, 0xa8d, 34}, {0xa91, 0xa8f, 34}, {0xaa8, 0xa93, 34}, {0xab0, 0xaaa, 34},
	{0xab3, 0xab2, 34}, {0xab9, 0xab5, 34}, {0xabd, 0xabd, 34}, {0xae0, 0xae0, 34},
	{0xb0c, 0xb05, 34}, {0xb10, 0xb0f, 34}, {0xb28, 0xb13, 34}, {0xb30, 0xb2a, 34},
	{0xb33, 0xb32, 34}, {0xb39, 0xb36, 34}, {0xb3d, 0xb3d, 34}, {0xb5d, 0xb5c, 34},
	{0xb61, 0xb5f, 34}, {0xb8a, 0xb85, 34}, {0xb90, 0xb8e, 34}, {0xb95, 0xb92, 34},
	{0xb9a, 0xb99, 34}, {0xb9c, 0xb9c, 34}, {0xb9f, 0xb9e, 34}, {0xba4, 0xba3, 34},
	{0xbaa, 0xba8, 34}, {0xbb5, 0xbae, 34}, {0xbb9, 0xbb7, 34}, {0xc0c, 0xc05, 34},
	{0xc10, 0xc0e, 34}, {0xc28, 0xc12, 34}, {0xc33, 0xc2a, 34}, {0xc39, 0xc35, 34},
	{0xc61, 0xc60, 34}, {0xc8c, 0xc85, 34}, {0xc90, 0xc8e, 34}, {0xca8, 0xc92, 34},
	{0xcb3, 0xcaa, 34}, {0xcb9, 0xcb5, 34}, {0xcde, 0xcde, 34}, {0xce1, 0xce0, 34},
	{0xd0c, 0xd05, 34}, {0xd10, 0xd0e, 34}, {0xd28, 0xd12, 34}, {0xd39, 0xd2a, 34},
	{0xd61, 0xd60, 34}, {0xe2e, 0xe01, 34}, {0xe30, 0xe30, 34}, {0xe33, 0xe32, 34},
	{0xe45, 0xe40, 34}, {0xe82, 0xe81, 34}, {0xe84, 0xe84, 34}, {0xe88, 0xe87, 34},
	{0xe8a, 0xe8a, 34}, {0xe8d, 0xe8d, 34}, {0xe97, 0xe94, 34}, {0xe9f, 0xe99, 34},
	{0xea3, 0xea1, 34}, {0xea5, 0xea5, 34}, {0xea7, 0xea7, 34}, {0xeab, 0xeaa, 34},
	{0xeae, 0xead, 34}, {0xeb0, 0xeb0, 34}, {0xeb3, 0xeb2, 34}, {0xebd, 0xebd, 34},
	{0xec4, 0xec0, 34}, {0xf47, 0xf40, 34}, {0xf69, 0xf49, 34}, {0x10c5, 0x10a0, 34},
	{0x10f6, 0x10d0, 34}, {0x1100, 0x1100, 34}, {0x1103, 0x1102, 34}, {0x1107, 0x1105, 34},
	{0x1109, 0x1109, 34}, {0x110c, 0x110b, 34}, {0x1112, 0x110e, 34}, {0x113c, 0x113c, 34},
	{0x113e, 0x113e, 34}, {0x1140, 0x1140, 34}, {0x114c, 0x114c, 34}, {0x114e, 0x114e, 34},
	{0x1150, 0x1150, 34}, {0x1155, 0x1154, 34}, {0x1159, 0x1159, 34}, {0x1161, 0x115f, 34},
	{0x1163, 0x1163, 34}, {0x1165, 0x1165, 34}, {0x1167, 0x1167, 34}, {0x1169, 0x1169, 34},
	{0x116e, 0x116d, 34}, {0x1173, 0x1172, 34}, {0x1175, 0x1175, 34}, {0x119e, 0x119e, 34},
	{0x11a8, 0x11a8, 34}, {0x11ab, 0x11ab, 34}, {0x11af, 0x11ae, 34}, {0x11b8, 0x11b7, 34},
	{0x11ba, 0x11ba, 34}, {0x11c2, 0x11bc, 34}, {0x11eb, 0x11eb, 34}, {0x11f0, 0x11f0, 34},
	{0x11f9, 0x11f9, 34}, {0x1e9b, 0x1e00, 34}, {0x1ef9, 0x1ea0, 34}, {0x1f15, 0x1f00, 34},
	{0x1f1d, 0x1f18, 34}, {0x1f45, 0x1f20, 34}, {0x1f4d, 0x1f48, 34}, {0x1f57, 0x1f50, 34},
	{0x1f59, 0x1f59, 34}, {0x1f5b, 0x1f5b, 34}, {0x1f5d, 0x1f5d, 34}, {0x1f7d, 0x1f5f, 34},
	{0x1fb4, 0x1f80, 34}, {0x1fbc, 0x1fb6, 34}, {0x1fbe, 0x1fbe, 34}, {0x1fc4, 0x1fc2, 34},
	{0x1fcc, 0x1fc6, 34}, {0x1fd3, 0x1fd0, 34}, {0x1fdb, 0x1fd6, 34}, {0x1fec, 0x1fe0, 34},
	{0x1ff4, 0x1ff2, 34}, {0x1ffc, 0x1ff6, 34}, {0x2126, 0x2126, 34}, {0x212b, 0x212a, 34},
	{0x212e, 0x212e, 34}, {0x2182, 0x2180, 34}, {0x3094, 0x3041, 34}, {0x30fa, 0x30a1, 34},
	{0x312c, 0x3105, 34}, {0xd7a3, 0xac00, 34}, {0x3007, 0x3007, 34}, {0x3029, 0x3021, 34},
	{0x9fa5, 0x4e00, 34}, {0x23, 0x23, 33}, {0x39, 0x30, 35}, {0x78, 0x78, 36}, {0x345, 0x300, 34},
	{0x361, 0x360, 34}, {0x486, 0x483, 34}, {0x5a1, 0x591, 34}, {0x5b9, 0x5a3, 34},
	{0x5bd, 0x5bb, 34}, {0x5bf, 0x5bf, 34}, {0x5c2, 0x5c1, 34}, {0x5c4, 0x5c4, 34},
	{0x652, 0x64b, 34}, {0x670, 0x670, 34}, {0x6e4, 0x6d6, 34}, {0x6e8, 0x6e7, 34},
	{0x6ed, 0x6ea, 34}, {0x903, 0x901, 34}, {0x93c, 0x93c, 34}, {0x94d, 0x93e, 34},
	{0x954, 0x951, 34}, {0x963, 0x962, 34}, {0x983, 0x981, 34}, {0x9bc, 0x9bc, 34},
	{0x9c4, 0x9be, 34}, {0x9c8, 0x9c7, 34}, {0x9cd, 0x9cb, 34}, {0x9d7, 0x9d7, 34},
	{0x9e3, 0x9e2, 34}, {0xa02, 0xa02, 34}, {0xa3c, 0xa3c, 34}, {0xa42, 0xa3e, 34},
	{0xa48, 0xa47, 34}, {0xa4d, 0xa4b, 34}, {0xa71, 0xa70, 34}, {0xa83, 0xa81, 34},
	{0xabc, 0xabc, 34}, {0xac5, 0xabe, 34}, {0xac9, 0xac7, 34}, {0xacd, 0xacb, 34},
	{0xb03, 0xb01, 34}, {0xb3c, 0xb3c, 34}, {0xb43, 0xb3e, 34}, {0xb48, 0xb47, 34},
	{0xb4d, 0xb4b, 34}, {0xb57, 0xb56, 34}, {0xb83, 0xb82, 34}, {0xbc2, 0xbbe, 34},
	{0xbc8, 0xbc6, 34}, {0xbcd, 0xbca, 34}, {0xbd7, 0xbd7, 34}, {0xc03, 0xc01, 34},
	{0xc44, 0xc3e, 34}, {0xc48, 0xc46, 34}, {0xc4d, 0xc4a, 34}, {0xc56, 0xc55, 34},
	{0xc83, 0xc82, 34}, {0xcc4, 0xcbe, 34}, {0xcc8, 0xcc6, 34}, {0xccd, 0xcca, 34},
	{0xcd6, 0xcd5, 34}, {0xd03, 0xd02, 34}, {0xd43, 0xd3e, 34}, {0xd48, 0xd46, 34},
	{0xd4d, 0xd4a, 34}, {0xd57, 0xd57, 34}, {0xe31, 0xe31, 34}, {0xe3a, 0xe34, 34},
	{0xe4e, 0xe47, 34}, {0xeb1, 0xeb1, 34}, {0xeb9, 0xeb4, 34}, {0xebc, 0xebb, 34},
	{0xecd, 0xec8, 34}, {0xf19, 0xf18, 34}, {0xf35, 0xf35, 34}, {0xf37, 0xf37, 34},
	{0xf39, 0xf39, 34}, {0xf3f, 0xf3e, 34}, {0xf84, 0xf71, 34}, {0xf8b, 0xf86, 34},
	{0xf95, 0xf90, 34}, {0xf97, 0xf97, 34}, {0xfad, 0xf99, 34}, {0xfb7, 0xfb1, 34},
	{0xfb9, 0xfb9, 34}, {0x20dc, 0x20d0, 34}, {0x20e1, 0x20e1, 34}, {0x302f, 0x302a, 34},
	{0x309a, 0x3099, 34}, {0x3007, 0x3007, 34}, {0x3029, 0x3021, 34}, {0x9fa5, 0x4e00, 34},
	{0x3b, 0x3b, 3}, {0x5a, 0x41, 34}, {0x7a, 0x61, 34}, {0xd6, 0xc0, 34}, {0xf6, 0xd8, 34},
	{0x131, 0xf8, 34}, {0x13e, 0x134, 34}, {0x148, 0x141, 34}, {0x17e, 0x14a, 34},
	{0x1c3, 0x180, 34}, {0x1f0, 0x1cd, 34}, {0x1f5, 0x1f4, 34}, {0x217, 0x1fa, 34},
	{0x2a8, 0x250, 34}, {0x2c1, 0x2bb, 34}, {0x386, 0x386, 34}, {0x38a, 0x388, 34},
	{0x38c, 0x38c, 34}, {0x3a1, 0x38e, 34}, {0x3ce, 0x3a3, 34}, {0x3d6, 0x3d0, 34},
	{0x3da, 0x3da, 34}, {0x3dc, 0x3dc, 34}, {0x3de, 0x3de, 34}, {0x3e0, 0x3e0, 34},
	{0x3f3, 0x3e2, 34}, {0x40c, 0x401, 34}, {0x44f, 0x40e, 34}, {0x45c, 0x451, 34},
	{0x481, 0x45e, 34}, {0x4c4, 0x490, 34}, {0x4c8, 0x4c7, 34}, {0x4cc, 0x4cb, 34},
	{0x4eb, 0x4d0, 34}, {0x4f5, 0x4ee, 34}, {0x4f9, 0x4f8, 34}, {0x556, 0x531, 34},
	{0x559, 0x559, 34}, {0x586, 0x561, 34}, {0x5ea, 0x5d0, 34}, {0x5f2, 0x5f0, 34},
	{0x63a, 0x621, 34}, {0x64a, 0x641, 34}, {0x6b7, 0x671, 34}, {0x6be, 0x6ba, 34},
	{0x6ce, 0x6c0, 34}, {0x6d3, 0x6d0, 34}, {0x6d5, 0x6d5, 34}, {0x6e6, 0x6e5, 34},
	{0x939, 0x905, 34}, {0x93d, 0x93d, 34}, {0x961, 0x958, 34}, {0x98c, 0x985, 34},
	{0x990, 0x98f, 34}, {0x9a8, 0x993, 34}, {0x9b0, 0x9aa, 34}, {0x9b2, 0x9b2, 34},
	{0x9b9, 0x9b6, 34}, {0x9dd, 0x9dc, 34}, {0x9e1, 0x9df, 34}, {0x9f1, 0x9f0, 34},
	{0xa0a, 0xa05, 34}, {0xa10, 0xa0f, 34}, {0xa28, 0xa13, 34}, {0xa30, 0xa2a, 34},
	{0xa33, 0xa32, 34}, {0xa36, 0xa35, 34}, {0xa39, 0xa38, 34}, {0xa5c, 0xa59, 34},
	{0xa5e, 0xa5e, 34}, {0xa74, 0xa72, 34}, {0xa8b, 0xa85, 34}, {0xa8d, 0xa8d, 34},
	{0xa91, 0xa8f, 34}, {0xaa8, 0xa93, 34}, {0xab0, 0xaaa, 34}, {0xab3, 0xab2, 34},
	{0xab9, 0xab5, 34}, {0xabd, 0xabd, 34}, {0xae0, 0xae0, 34}, {0xb0c, 0xb05, 34},
	{0xb10, 0xb0f, 34}, {0xb28, 0xb13, 34}, {0xb30, 0xb2a, 34}, {0xb33, 0xb32, 34},
	{0xb39, 0xb36, 34}, {0xb3d, 0xb3d, 34}, {0xb5d, 0xb5c, 34}, {0xb61, 0xb5f, 34},
	{0xb8a, 0xb85, 34}, {0xb90, 0xb8e, 34}, {0xb95, 0xb92, 34}, {0xb9a, 0xb99, 34},
	{0xb9c, 0xb9c, 34}, {0xb9f, 0xb9e, 34}, {0xba4, 0xba3, 34}, {0xbaa, 0xba8, 34},
	{0xbb5, 0xbae, 34}, {0xbb9, 0xbb7, 34}, {0xc0c, 0xc05, 34}, {0xc10, 0xc0e, 34},
	{0xc28, 0xc12, 34}, {0xc33, 0xc2a, 34}, {0xc39, 0xc35, 34}, {0xc61, 0xc60, 34},
	{0xc8c, 0xc85, 34}, {0xc90, 0xc8e, 34}, {0xca8, 0xc92, 34}, {0xcb3, 0xcaa, 34},
	{0xcb9, 0xcb5, 34}, {0xcde, 0xcde, 34}, {0xce1, 0xce0, 34}, {0xd0c, 0xd05, 34},
	{0xd10, 0xd0e, 34}, {0xd28, 0xd12, 34}, {0xd39, 0xd2a, 34}, {0xd61, 0xd60, 34},
	{0xe2e, 0xe01, 34}, {0xe30, 0xe30, 34}, {0xe33, 0xe32, 34}, {0xe45, 0xe40, 34},
	{0xe82, 0xe81, 34}, {0xe84, 0xe84, 34}, {0xe88, 0xe87, 34}, {0xe8a, 0xe8a, 34},
	{0xe8d, 0xe8d, 34}, {0xe97, 0xe94, 34}, {0xe9f, 0xe99, 34}, {0xea3, 0xea1, 34},
	{0xea5, 0xea5, 34}, {0xea7, 0xea7, 34}, {0xeab, 0xeaa, 34}, {0xeae, 0xead, 34},
	{0xeb0, 0xeb0, 34}, {0xeb3, 0xeb2, 34}, {0xebd, 0xebd, 34}, {0xec4, 0xec0, 34},
	{0xf47, 0xf40, 34}, {0xf69, 0xf49, 34}, {0x10c5, 0x10a0, 34}, {0x10f6, 0x10d0, 34},
	{0x1100, 0x1100, 34}, {0x1103, 0x1102, 34}, {0x1107, 0x1105, 34}, {0x1109, 0x1109, 34},
	{0x110c, 0x110b, 34}, {0x1112, 0x110e, 34}, {0x113c, 0x113c, 34}, {0x113e, 0x113e, 34},
	{0x1140, 0x1140, 34}, {0x114c, 0x114c, 34}, {0x114e, 0x114e, 34}, {0x1150, 0x1150, 34},
	{0x1155, 0x1154, 34}, {0x1159, 0x1159, 34}, {0x1161, 0x115f, 34}, {0x1163, 0x1163, 34},
	{0x1165, 0x1165, 34}, {0x1167, 0x1167, 34}, {0x1169, 0x1169, 34}, {0x116e, 0x116d, 34},
	{0x1173, 0x1172, 34}, {0x1175, 0x1175, 34}, {0x119e, 0x119e, 34}, {0x11a8, 0x11a8, 34},
	{0x11ab, 0x11ab, 34}, {0x11af, 0x11ae, 34}, {0x11b8, 0x11b7, 34}, {0x11ba, 0x11ba, 34},
	{0x11c2, 0x11bc, 34}, {0x11eb, 0x11eb, 34}, {0x11f0, 0x11f0, 34}, {0x11f9, 0x11f9, 34},
	{0x1e9b, 0x1e00, 34}, {0x1ef9, 0x1ea0, 34}, {0x1f15, 0x1f00, 34}, {0x1f1d, 0x1f18, 34},
	{0x1f45, 0x1f20, 34}, {0x1f4d, 0x1f48, 34}, {0x1f57, 0x1f50, 34}, {0x1f59, 0x1f59, 34},
	{0x1f5b, 0x1f5b, 34}, {0x1f5d, 0x1f5d, 34}, {0x1f7d, 0x1f5f, 34}, {0x1fb4, 0x1f80, 34},
	{0x1fbc, 0x1fb6, 34}, {0x1fbe, 0x1fbe, 34}, {0x1fc4, 0x1fc2, 34}, {0x1fcc, 0x1fc6, 34},
	{0x1fd3, 0x1fd0, 34}, {0x1fdb, 0x1fd6, 34}, {0x1fec, 0x1fe0, 34}, {0x1ff4, 0x1ff2, 34},
	{0x1ffc, 0x1ff6, 34}, {0x2126, 0x2126, 34}, {0x212b, 0x212a, 34}, {0x212e, 0x212e, 34},
	{0x2182, 0x2180, 34}, {0x3094, 0x3041, 34}, {0x30fa, 0x30a1, 34}, {0x312c, 0x3105, 34},
	{0xd7a3, 0xac00, 34}, {0xb7, 0xb7, 34}, {0x2d1, 0x2d0, 34}, {0x387, 0x387, 34},
	{0x640, 0x640, 34}, {0xe46, 0xe46, 34}, {0xec6, 0xec6, 34}, {0x3005, 0x3005, 34},
	{0x3035, 0x3031, 34}, {0x309e, 0x309d, 34}, {0x30fe, 0x30fc, 34}, {0x2e, 0x2d, 34},
	{0x3a, 0x3a, 34}, {0x5f, 0x5f, 34}, {0x39, 0x30, 34}, {0x669, 0x660, 34}, {0x6f9, 0x6f0, 34},
	{0x96f, 0x966, 34}, {0x9ef, 0x9e6, 34}, {0xa6f, 0xa66, 34}, {0xaef, 0xae6, 34},
	{0xb6f, 0xb66, 34}, {0xbef, 0xbe7, 34}, {0xc6f, 0xc66, 34}, {0xcef, 0xce6, 34},
	{0xd6f, 0xd66, 34}, {0xe59, 0xe50, 34}, {0xed9, 0xed0, 34}, {0xf29, 0xf20, 34},
	{0x39, 0x30, 35}, {0x3b, 0x3b, 3}, {0x39, 0x30, 37}, {0x46, 0x41, 37}, {0x66, 0x61, 37},
	{0x39, 0x30, 37}, {0x46, 0x41, 37}, {0x66, 0x61, 37}, {0x3b, 0x3b, 3}, {0x3a, 0x3a, 40},
	{0x5f, 0x5f, 40}, {0x5a, 0x41, 40}, {0x7a, 0x61, 40}, {0xd6, 0xc0, 40}, {0xf6, 0xd8, 40},
	{0x131, 0xf8, 40}, {0x13e, 0x134, 40}, {0x148, 0x141, 40}, {0x17e, 0x14a, 40},
	{0x1c3, 0x180, 40}, {0x1f0, 0x1cd, 40}, {0x1f5, 0x1f4, 40}, {0x217, 0x1fa, 40},
	{0x2a8, 0x250, 40}, {0x2c1, 0x2bb, 40}, {0x386, 0x386, 40}, {0x38a, 0x388, 40},
	{0x38c, 0x38c, 40}, {0x3a1, 0x38e, 40}, {0x3ce, 0x3a3, 40}, {0x3d6, 0x3d0, 40},
	{0x3da, 0x3da, 40}, {0x3dc, 0x3dc, 40}, {0x3de, 0x3de, 40}, {0x3e0, 0x3e0, 40},
	{0x3f3, 0x3e2, 40}, {0x40c, 0x401, 40}, {0x44f, 0x40e, 40}, {0x45c, 0x451, 40},
	{0x481, 0x45e, 40}, {0x4c4, 0x490, 40}, {0x4c8, 0x4c7, 40}, {0x4cc, 0x4cb, 40},
	{0x4eb, 0x4d0, 40}, {0x4f5, 0x4ee, 40}, {0x4f9, 0x4f8, 40}, {0x556, 0x531, 40},
	{0x559, 0x559, 40}, {0x586, 0x561, 40}, {0x5ea, 0x5d0, 40}, {0x5f2, 0x5f0, 40},
	{0x63a, 0x621, 40}, {0x64a, 0x641, 40}, {0x6b7, 0x671, 40}, {0x6be, 0x6ba, 40},
	{0x6ce, 0x6c0, 40}, {0x6d3, 0x6d0, 40}, {0x6d5, 0x6d5, 40}, {0x6e6, 0x6e5, 40},
	{0x939, 0x905, 40}, {0x93d, 0x93d, 40}, {0x961, 0x958, 40}, {0x98c, 0x985, 40},
	{0x990, 0x98f, 40}, {0x9a8, 0x993, 40}, {0x9b0, 0x9aa, 40}, {0x9b2, 0x9b2, 40},
	{0x9b9, 0x9b6, 40}, {0x9dd, 0x9dc, 40}, {0x9e1, 0x9df, 40}, {0x9f1, 0x9f0, 40},
	{0xa0a, 0xa05, 40}, {0xa10, 0xa0f, 40}, {0xa28, 0xa13, 40}, {0xa30, 0xa2a, 40},
	{0xa33, 0xa32, 40}, {0xa36, 0xa35, 40}, {0xa39, 0xa38, 40}, {0xa5c, 0xa59, 40},
	{0xa5e, 0xa5e, 40}, {0xa74, 0xa72, 40}, {0xa8b, 0xa85, 40}, {0xa8d, 0xa8d, 40},
	{0xa91, 0xa8f, 40}, {0xaa8, 0xa93, 40}, {0xab0, 0xaaa, 40}, {0xab3, 0xab2, 40},
	{0xab9, 0xab5, 40}, {0xabd, 0xabd, 40}, {0xae0, 0xae0, 40}, {0xb0c, 0xb05, 40},
	{0xb10, 0xb0f, 40}, {0xb28, 0xb13, 40}, {0xb30, 0xb2a, 40}, {0xb33, 0xb32, 40},
	{0xb39, 0xb36, 40}, {0xb3d, 0xb3d, 40}, {0xb5d, 0xb5c, 40}, {0xb61, 0xb5f, 40},
	{0xb8a, 0xb85, 40}, {0xb90, 0xb8e, 40}, {0xb95, 0xb92, 40}, {0xb9a, 0xb99, 40},
	{0xb9c, 0xb9c, 40}, {0xb9f, 0xb9e, 40}, {0xba4, 0xba3, 40}, {0xbaa, 0xba8, 40},
	{0xbb5, 0xbae, 40}, {0xbb9, 0xbb7, 40}, {0xc0c, 0xc05, 40}, {0xc10, 0xc0e, 40},
	{0xc28, 0xc12, 40}, {0xc33, 0xc2a, 40}, {0xc39, 0xc35, 40}, {0xc61, 0xc60, 40},
	{0xc8c, 0xc85, 40}, {0xc90, 0xc8e, 40}, {0xca8, 0xc92, 40}, {0xcb3, 0xcaa, 40},
	{0xcb9, 0xcb5, 40}, {0xcde, 0xcde, 40}, {0xce1, 0xce0, 40}, {0xd0c, 0xd05, 40},
	{0xd10, 0xd0e, 40}, {0xd28, 0xd12, 40}, {0xd39, 0xd2a, 40}, {0xd61, 0xd60, 40},
	{0xe2e, 0xe01, 40}, {0xe30, 0xe30, 40}, {0xe33, 0xe32, 40}, {0xe45, 0xe40, 40},
	{0xe82, 0xe81, 40}, {0xe84, 0xe84, 40}, {0xe88, 0xe87, 40}, {0xe8a, 0xe8a, 40},
	{0xe8d, 0xe8d, 40}, {0xe97, 0xe94, 40}, {0xe9f, 0xe99, 40}, {0xea3, 0xea1, 40},
	{0xea5, 0xea5, 40}, {0xea7, 0xea7, 40}, {0xeab, 0xeaa, 40}, {0xeae, 0xead, 40},
	{0xeb0, 0xeb0, 40}, {0xeb3, 0xeb2, 40}, {0xebd, 0xebd, 40}, {0xec4, 0xec0, 40},
	{0xf47, 0xf40, 40}, {0xf69, 0xf49, 40}, {0x10c5, 0x10a0, 40}, {0x10f6, 0x10d0, 40},
	{0x1100, 0x1100, 40}, {0x1103, 0x1102, 40}, {0x1107, 0x1105, 40}, {0x1109, 0x1109, 40},
	{0x110c, 0x110b, 40}, {0x1112, 0x110e, 40}, {0x113c, 0x113c, 40}, {0x113e, 0x113e, 40},
	{0x1140, 0x1140, 40}, {0x114c, 0x114c, 40}, {0x114e, 0x114e, 40}, {0x1150, 0x1150, 40},
	{0x1155, 0x1154, 40}, {0x1159, 0x1159, 40}, {0x1161, 0x115f, 40}, {0x1163, 0x1163, 40},
	{0x1165, 0x1165, 40}, {0x1167, 0x1167, 40}, {0x1169, 0x1169, 40}, {0x116e, 0x116d, 40},
	{0x1173, 0x1172, 40}, {0x1175, 0x1175, 40}, {0x119e, 0x119e, 40}, {0x11a8, 0x11a8, 40},
	{0x11ab, 0x11ab, 40}, {0x11af, 0x11ae, 40}, {0x11b8, 0x11b7, 40}, {0x11ba, 0x11ba, 40},
	{0x11c2, 0x11bc, 40}, {0x11eb, 0x11eb, 40}, {0x11f0, 0x11f0, 40}, {0x11f9, 0x11f9, 40},
	{0x1e9b, 0x1e00, 40}, {0x1ef9, 0x1ea0, 40}, {0x1f15, 0x1f00, 40}, {0x1f1d, 0x1f18, 40},
	{0x1f45, 0x1f20, 40}, {0x1f4d, 0x1f48, 40}, {0x1f57, 0x1f50, 40}, {0x1f59, 0x1f59, 40},
	{0x1f5b, 0x1f5b, 40}, {0x1f5d, 0x1f5d, 40}, {0x1f7d, 0x1f5f, 40}, {0x1fb4, 0x1f80, 40},
	{0x1fbc, 0x1fb6, 40}, {0x1fbe, 0x1fbe, 40}, {0x1fc4, 0x1fc2, 40}, {0x1fcc, 0x1fc6, 40},
	{0x1fd3, 0x1fd0, 40}, {0x1fdb, 0x1fd6, 40}, {0x1fec, 0x1fe0, 40}, {0x1ff4, 0x1ff2, 40},
	{0x1ffc, 0x1ff6, 40}, {0x2126, 0x2126, 40}, {0x212b, 0x212a, 40}, {0x212e, 0x212e, 40},
	{0x2182, 0x2180, 40}, {0x3094, 0x3041, 40}, {0x30fa, 0x30a1, 40}, {0x312c, 0x3105, 40},
	{0xd7a3, 0xac00, 40}, {0x23, 0x23, 39}, {0x3007, 0x3007, 40}, {0x3029, 0x3021, 40},
	{0x9fa5, 0x4e00, 40}, {0x78, 0x78, 42}, {0x39, 0x30, 41}, {0x5a, 0x41, 40}, {0x7a, 0x61, 40},
	{0xd6, 0xc0, 40}, {0xf6, 0xd8, 40}, {0x131, 0xf8, 40}, {0x13e, 0x134, 40}, {0x148, 0x141, 40},
	{0x17e, 0x14a, 40}, {0x1c3, 0x180, 40}, {0x1f0, 0x1cd, 40}, {0x1f5, 0x1f4, 40},
	{0x217, 0x1fa, 40}, {0x2a8, 0x250, 40}, {0x2c1, 0x2bb, 40}, {0x386, 0x386, 40},
	{0x38a, 0x388, 40}, {0x38c, 0x38c, 40}, {0x3a1, 0x38e, 40}, {0x3ce, 0x3a3, 40},
	{0x3d6, 0x3d0, 40}, {0x3da, 0x3da, 40}, {0x3dc, 0x3dc, 40}, {0x3de, 0x3de, 40},
	{0x3e0, 0x3e0, 40}, {0x3f3, 0x3e2, 40}, {0x40c, 0x401, 40}, {0x44f, 0x40e, 40},
	{0x45c, 0x451, 40}, {0x481, 0x45e, 40}, {0x4c4, 0x490, 40}, {0x4c8, 0x4c7, 40},
	{0x4cc, 0x4cb, 40}, {0x4eb, 0x4d0, 40}, {0x4f5, 0x4ee, 40}, {0x4f9, 0x4f8, 40},
	{0x556, 0x531, 40}, {0x559, 0x559, 40}, {0x586, 0x561, 40}, {0x5ea, 0x5d0, 40},
	{0x5f2, 0x5f0, 40}, {0x63a, 0x621, 40}, {0x64a, 0x641, 40}, {0x6b7, 0x671, 40},
	{0x6be, 0x6ba, 40}, {0x6ce, 0x6c0, 40}, {0x6d3, 0x6d0, 40}, {0x6d5, 0x6d5, 40},
	{0x6e6, 0x6e5, 40}, {0x939, 0x905, 40}, {0x93d, 0x93d, 40}, {0x961, 0x958, 40},
	{0x98c, 0x985, 40}, {0x990, 0x98f, 40}, {0x9a8, 0x993, 40}, {0x9b0, 0x9aa, 40},
	{0x9b2, 0x9b2, 40}, {0x9b9, 0x9b6, 40}, {0x9dd, 0x9dc, 40}, {0x9e1, 0x9df, 40},
	{0x9f1, 0x9f0, 40}, {0xa0a, 0xa05, 40}, {0xa10, 0xa0f, 40}, {0xa28, 0xa13, 40},
	{0xa30, 0xa2a, 40}, {0xa33, 0xa32, 40}, {0xa36, 0xa35, 40}, {0xa39, 0xa38, 40},
	{0xa5c, 0xa59, 40}, {0xa5e, 0xa5e, 40}, {0xa74, 0xa72, 40}, {0xa8b, 0xa85, 40},
	{0xa8d, 0xa8d, 40}, {0xa91, 0xa8f, 40}, {0xaa8, 0xa93, 40}, {0xab0, 0xaaa, 40},
	{0xab3, 0xab2, 40}, {0xab9, 0xab5, 40}, {0xabd, 0xabd, 40}, {0xae0, 0xae0, 40},
	{0xb0c, 0xb05, 40}, {0xb10, 0xb0f, 40}, {0xb28, 0xb13, 40}, {0xb30, 0xb2a, 40},
	{0xb33, 0xb32, 40}, {0xb39, 0xb36, 40}, {0xb3d, 0xb3d, 40}, {0xb5d, 0xb5c, 40},
	{0xb61, 0xb5f, 40}, {0xb8a, 0xb85, 40}, {0xb90, 0xb8e, 40}, {0xb95, 0xb92, 40},
	{0xb9a, 0xb99, 40}, {0xb9c, 0xb9c, 40}, {0xb9f, 0xb9e, 40}, {0xba4, 0xba3, 40},
	{0xbaa, 0xba8, 40}, {0xbb5, 0xbae, 40}, {0xbb9, 0xbb7, 40}, {0xc0c, 0xc05, 40},
	{0xc10, 0xc0e, 40}, {0xc28, 0xc12, 40}, {0xc33, 0xc2a, 40}, {0xc39, 0xc35, 40},
	{0xc61, 0xc60, 40}, {0xc8c, 0xc85, 40}, {0xc90, 0xc8e, 40}, {0xca8, 0xc92, 40},
	{0xcb3, 0xcaa, 40}, {0xcb9, 0xcb5, 40}, {0xcde, 0xcde, 40}, {0xce1, 0xce0, 40},
	{0xd0c, 0xd05, 40}, {0xd10, 0xd0e, 40}, {0xd28, 0xd12, 40}, {0xd39, 0xd2a, 40},
	{0xd61, 0xd60, 40}, {0xe2e, 0xe01, 40}, {0xe30, 0xe30, 40}, {0xe33, 0xe32, 40},
	{0xe45, 0xe40, 40}, {0xe82, 0xe81, 40}, {0xe84, 0xe84, 40}, {0xe88, 0xe87, 40},
	{0xe8a, 0xe8a, 40}, {0xe8d, 0xe8d, 40}, {0xe97, 0xe94, 40}, {0xe9f, 0xe99, 40},
	{0xea3, 0xea1, 40}, {0xea5, 0xea5, 40}, {0xea7, 0xea7, 40}, {0xeab, 0xeaa, 40},
	{0xeae, 0xead, 40}, {0xeb0, 0xeb0, 40}, {0xeb3, 0xeb2, 40}, {0xebd, 0xebd, 40},
	{0xec4, 0xec0, 40}, {0xf47, 0xf40, 40}, {0xf69, 0xf49, 40}, {0x10c5, 0x10a0, 40},
	{0x10f6, 0x10d0, 40}, {0x1100, 0x1100, 40}, {0x1103, 0x1102, 40}, {0x1107, 0x1105, 40},
	{0x1109, 0x1109, 40}, {0x110c, 0x110b, 40}, {0x1112, 0x110e, 40}, {0x113c, 0x113c, 40},
	{0x113e, 0x113e, 40}, {0x1140, 0x1140, 40}, {0x114c, 0x114c, 40}, {0x114e, 0x114e, 40},
	{0x1150, 0x1150, 40}, {0x1155, 0x1154, 40}, {0x1159, 0x1159, 40}, {0x1161, 0x115f, 40},
	{0x1163, 0x1163, 40}, {0x1165, 0x1165, 40}, {0x1167, 0x1167, 40}, {0x1169, 0x1169, 40},
	{0x116e, 0x116d, 40}, {0x1173, 0x1172, 40}, {0x1175, 0x1175, 40}, {0x119e, 0x119e, 40},
	{0x11a8, 0x11a8, 40}, {0x11ab, 0x11ab, 40}, {0x11af, 0x11ae, 40}, {0x11b8, 0x11b7, 40},
	{0x11ba, 0x11ba, 40}, {0x11c2, 0x11bc, 40}, {0x11eb, 0x11eb, 40}, {0x11f0, 0x11f0, 40},
	{0x11f9, 0x11f9, 40}, {0x1e9b, 0x1e00, 40}, {0x1ef9, 0x1ea0, 40}, {0x1f15, 0x1f00, 40},
	{0x1f1d, 0x1f18, 40}, {0x1f45, 0x1f20, 40}, {0x1f4d, 0x1f48, 40}, {0x1f57, 0x1f50, 40},
	{0x1f59, 0x1f59, 40}, {0x1f5b, 0x1f5b, 40}, {0x1f5d, 0x1f5d, 40}, {0x1f7d, 0x1f5f, 40},
	{0x1fb4, 0x1f80, 40}, {0x1fbc, 0x1fb6, 40}, {0x1fbe, 0x1fbe, 40}, {0x1fc4, 0x1fc2, 40},
	{0x1fcc, 0x1fc6, 40}, {0x1fd3, 0x1fd0, 40}, {0x1fdb, 0x1fd6, 40}, {0x1fec, 0x1fe0, 40},
	{0x1ff4, 0x1ff2, 40}, {0x1ffc, 0x1ff6, 40}, {0x2126, 0x2126, 40}, {0x212b, 0x212a, 40},
	{0x212e, 0x212e, 40}, {0x2182, 0x2180, 40}, {0x3094, 0x3041, 40}, {0x30fa, 0x30a1, 40},
	{0x312c, 0x3105, 40}, {0xd7a3, 0xac00, 40}, {0xb7, 0xb7, 40}, {0x2d1, 0x2d0, 40},
	{0x387, 0x387, 40}, {0x640, 0x640, 40}, {0xe46, 0xe46, 40}, {0xec6, 0xec6, 40},
	{0x3005, 0x3005, 40}, {0x3035, 0x3031, 40}, {0x309e, 0x309d, 40}, {0x30fe, 0x30fc, 40},
	{0x345, 0x300, 40}, {0x361, 0x360, 40}, {0x486, 0x483, 40}, {0x5a1, 0x591, 40},
	{0x5b9, 0x5a3, 40}, {0x5bd, 0x5bb, 40}, {0x5bf, 0x5bf, 40}, {0x5c2, 0x5c1, 40},
	{0x5c4, 0x5c4, 40}, {0x652, 0x64b, 40}, {0x670, 0x670, 40}, {0x6e4, 0x6d6, 40},
	{0x6e8, 0x6e7, 40}, {0x6ed, 0x6ea, 40}, {0x903, 0x901, 40}, {0x93c, 0x93c, 40},
	{0x94d, 0x93e, 40}, {0x954, 0x951, 40}, {0x963, 0x962, 40}, {0x983, 0x981, 40},
	{0x9bc, 0x9bc, 40}, {0x9c4, 0x9be, 40}, {0x9c8, 0x9c7, 40}, {0x9cd, 0x9cb, 40},
	{0x9d7, 0x9d7, 40}, {0x9e3, 0x9e2, 40}, {0xa02, 0xa02, 40}, {0xa3c, 0xa3c, 40},
	{0xa42, 0xa3e, 40}, {0xa48, 0xa47, 40}, {0xa4d, 0xa4b, 40}, {0xa71, 0xa70, 40},
	{0xa83, 0xa81, 40}, {0xabc, 0xabc, 40}, {0xac5, 0xabe, 40}, {0xac9, 0xac7, 40},
	{0xacd, 0xacb, 40}, {0xb03, 0xb01, 40}, {0xb3c, 0xb3c, 40}, {0xb43, 0xb3e, 40},
	{0xb48, 0xb47, 40}, {0xb4d, 0xb4b, 40}, {0xb57, 0xb56, 40}, {0xb83, 0xb82, 40},
	{0xbc2, 0xbbe, 40}, {0xbc8, 0xbc6, 40}, {0xbcd, 0xbca, 40}, {0xbd7, 0xbd7, 40},
	{0xc03, 0xc01, 40}, {0xc44, 0xc3e, 40}, {0xc48, 0xc46, 40}, {0xc4d, 0xc4a, 40},
	{0xc56, 0xc55, 40}, {0xc83, 0xc82, 40}, {0xcc4, 0xcbe, 40}, {0xcc8, 0xcc6, 40},
	{0xccd, 0xcca, 40}, {0xcd6, 0xcd5, 40}, {0xd03, 0xd02, 40}, {0xd43, 0xd3e, 40},
	{0xd48, 0xd46, 40}, {0xd4d, 0xd4a, 40}, {0xd57, 0xd57, 40}, {0xe31, 0xe31, 40},
	{0xe3a, 0xe34, 40}, {0xe4e, 0xe47, 40}, {0xeb1, 0xeb1, 40}, {0xeb9, 0xeb4, 40},
	{0xebc, 0xebb, 40}, {0xecd, 0xec8, 40}, {0xf19, 0xf18, 40}, {0xf35, 0xf35, 40},
	{0xf37, 0xf37, 40}, {0xf39, 0xf39, 40}, {0xf3f, 0xf3e, 40}, {0xf84, 0xf71, 40},
	{0xf8b, 0xf86, 40}, {0xf95, 0xf90, 40}, {0xf97, 0xf97, 40}, {0xfad, 0xf99, 40},
	{0xfb7, 0xfb1, 40}, {0xfb9, 0xfb9, 40}, {0x20dc, 0x20d0, 40}, {0x20e1, 0x20e1, 40},
	{0x302f, 0x302a, 40}, {0x309a, 0x3099, 40}, {0x3007, 0x3007, 40}, {0x3029, 0x3021, 40},
	{0x9fa5, 0x4e00, 40}, {0x2e, 0x2d, 40}, {0x3a, 0x3a, 40}, {0x5f, 0x5f, 40}, {0x39, 0x30, 40},
	{0x669, 0x660, 40}, {0x6f9, 0x6f0, 40}, {0x96f, 0x966, 40}, {0x9ef, 0x9e6, 40},
	{0xa6f, 0xa66, 40}, {0xaef, 0xae6, 40}, {0xb6f, 0xb66, 40}, {0xbef, 0xbe7, 40},
	{0xc6f, 0xc66, 40}, {0xcef, 0xce6, 40}, {0xd6f, 0xd66, 40}, {0xe59, 0xe50, 40},
	{0xed9, 0xed0, 40}, {0xf29, 0xf20, 40}, {0x3b, 0x3b, 2}, {0x3b, 0x3b, 2}, {0x39, 0x30, 41},
	{0x39, 0x30, 43}, {0x46, 0x41, 43}, {0x66, 0x61, 43}, {0x3b, 0x3b, 2}, {0x39, 0x30, 43},
	{0x46, 0x41, 43}, {0x66, 0x61, 43}
};
const CXML_Lexer::TState CXML_Lexer::m_p_state[44] = {
	{m_p_transition + 0, -1, 216}, {m_p_transition + 216, 0, 3}, {m_p_transition + 219, -1, 6},
	{m_p_transition + 225, -1, 5}, {m_p_transition + 230, -1, 1}, {m_p_transition + 231, 11, 318},
	{m_p_transition + 549, 3, 3}, {m_p_transition + 552, 10, 0}, {m_p_transition + 552, 7, 0},
	{m_p_transition + 552, -1, 1}, {m_p_transition + 553, 9, 0}, {m_p_transition + 553, 6, 2},
	{m_p_transition + 555, 4, 0}, {m_p_transition + 555, 5, 0}, {m_p_transition + 555, -1, 1},
	{m_p_transition + 556, -1, 1}, {m_p_transition + 557, -1, 1}, {m_p_transition + 558, -1, 1},
	{m_p_transition + 559, -1, 1}, {m_p_transition + 560, -1, 1}, {m_p_transition + 561, -1, 1},
	{m_p_transition + 562, -1, 3}, {m_p_transition + 565, -1, 3}, {m_p_transition + 568, -1, 3},
	{m_p_transition + 571, 2, 0}, {m_p_transition + 571, -1, 2}, {m_p_transition + 573, -1, 3},
	{m_p_transition + 576, -1, 3}, {m_p_transition + 579, -1, 1}, {m_p_transition + 580, 1, 0},
	{m_p_transition + 580, 8, 0}, {m_p_transition + 580, -1, 207}, {m_p_transition + 787, 12, 0},
	{m_p_transition + 787, -1, 2}, {m_p_transition + 789, -1, 319}, {m_p_transition + 1108, -1, 2},
	{m_p_transition + 1110, -1, 3}, {m_p_transition + 1113, -1, 4},
	{m_p_transition + 1117, -1, 207}, {m_p_transition + 1324, -1, 2},
	{m_p_transition + 1326, -1, 319}, {m_p_transition + 1645, -1, 2},
	{m_p_transition + 1647, -1, 3}, {m_p_transition + 1650, -1, 4}
};

#else // __XML_LEXER_XML10_COMPLIANT_CHARSET
//#include "XML_LexerTables_Tiny.h" // m_p_transition, m_p_state

const CXML_Lexer::TTransition CXML_Lexer::m_p_transition[112] = {
	{0x3d, 0x3d, 7}, {0x2f, 0x2f, 4}, {0x3f, 0x3f, 9}, {0x3c, 0x3c, 6}, {0x5a, 0x41, 5},
	{0x7a, 0x61, 5}, {0x3e, 0x3e, 8}, {0x3a, 0x3a, 5}, {0x5f, 0x5f, 5}, {0x27, 0x27, 3},
	{0x22, 0x22, 2}, {0xa, 0x9, 1}, {0xd, 0xd, 1}, {0x20, 0x20, 1}, {0xa, 0x9, 1}, {0xd, 0xd, 1},
	{0x20, 0x20, 1}, {0x21, 0x0, 2}, {0x25, 0x23, 2}, {0x3b, 0x27, 2}, {0x10ffff, 0x3d, 2},
	{0x22, 0x22, 32}, {0x26, 0x26, 38}, {0x27, 0x27, 32}, {0x25, 0x0, 3}, {0x3b, 0x28, 3},
	{0x10ffff, 0x3d, 3}, {0x26, 0x26, 31}, {0x3e, 0x3e, 30}, {0x5a, 0x41, 5}, {0x7a, 0x61, 5},
	{0x39, 0x30, 5}, {0x2e, 0x2d, 5}, {0x3a, 0x3a, 5}, {0x5f, 0x5f, 5}, {0x2f, 0x2f, 12},
	{0x3f, 0x3f, 13}, {0x21, 0x21, 11}, {0x3e, 0x3e, 10}, {0x2d, 0x2d, 14}, {0x5b, 0x5b, 15},
	{0x2d, 0x2d, 25}, {0x43, 0x43, 16}, {0x44, 0x44, 17}, {0x41, 0x41, 18}, {0x54, 0x54, 19},
	{0x41, 0x41, 20}, {0x5b, 0x5b, 21}, {0x5c, 0x0, 21}, {0x10ffff, 0x5e, 21}, {0x5d, 0x5d, 22},
	{0x5c, 0x0, 21}, {0x10ffff, 0x5e, 21}, {0x5d, 0x5d, 23}, {0x3d, 0x0, 21}, {0x10ffff, 0x3f, 21},
	{0x3e, 0x3e, 24}, {0x2c, 0x0, 26}, {0x10ffff, 0x2e, 26}, {0x2d, 0x2d, 27}, {0x2c, 0x0, 26},
	{0x10ffff, 0x2e, 26}, {0x2d, 0x2d, 28}, {0x2c, 0x0, 26}, {0x10ffff, 0x2e, 26},
	{0x3e, 0x3e, 29}, {0x23, 0x23, 33}, {0x3a, 0x3a, 34}, {0x5f, 0x5f, 34}, {0x5a, 0x41, 34},
	{0x7a, 0x61, 34}, {0x39, 0x30, 35}, {0x78, 0x78, 36}, {0x2e, 0x2d, 34}, {0x3a, 0x3a, 34},
	{0x5f, 0x5f, 34}, {0x39, 0x30, 34}, {0x3b, 0x3b, 3}, {0x5a, 0x41, 34}, {0x7a, 0x61, 34},
	{0x39, 0x30, 35}, {0x3b, 0x3b, 3}, {0x39, 0x30, 37}, {0x46, 0x41, 37}, {0x66, 0x61, 37},
	{0x3b, 0x3b, 3}, {0x39, 0x30, 37}, {0x46, 0x41, 37}, {0x66, 0x61, 37}, {0x23, 0x23, 39},
	{0x3a, 0x3a, 40}, {0x5f, 0x5f, 40}, {0x5a, 0x41, 40}, {0x7a, 0x61, 40}, {0x78, 0x78, 42},
	{0x39, 0x30, 41}, {0x5a, 0x41, 40}, {0x7a, 0x61, 40}, {0x2e, 0x2d, 40}, {0x3a, 0x3a, 40},
	{0x5f, 0x5f, 40}, {0x39, 0x30, 40}, {0x3b, 0x3b, 2}, {0x39, 0x30, 41}, {0x3b, 0x3b, 2},
	{0x39, 0x30, 43}, {0x46, 0x41, 43}, {0x66, 0x61, 43}, {0x3b, 0x3b, 2}, {0x39, 0x30, 43},
	{0x46, 0x41, 43}, {0x66, 0x61, 43}
};
const CXML_Lexer::TState CXML_Lexer::m_p_state[44] = {
	{m_p_transition + 0, -1, 14}, {m_p_transition + 14, 0, 3}, {m_p_transition + 17, -1, 6},
	{m_p_transition + 23, -1, 5}, {m_p_transition + 28, -1, 1}, {m_p_transition + 29, 11, 6},
	{m_p_transition + 35, 3, 3}, {m_p_transition + 38, 10, 0}, {m_p_transition + 38, 7, 0},
	{m_p_transition + 38, -1, 1}, {m_p_transition + 39, 9, 0}, {m_p_transition + 39, 6, 2},
	{m_p_transition + 41, 4, 0}, {m_p_transition + 41, 5, 0}, {m_p_transition + 41, -1, 1},
	{m_p_transition + 42, -1, 1}, {m_p_transition + 43, -1, 1}, {m_p_transition + 44, -1, 1},
	{m_p_transition + 45, -1, 1}, {m_p_transition + 46, -1, 1}, {m_p_transition + 47, -1, 1},
	{m_p_transition + 48, -1, 3}, {m_p_transition + 51, -1, 3}, {m_p_transition + 54, -1, 3},
	{m_p_transition + 57, 2, 0}, {m_p_transition + 57, -1, 2}, {m_p_transition + 59, -1, 3},
	{m_p_transition + 62, -1, 3}, {m_p_transition + 65, -1, 1}, {m_p_transition + 66, 1, 0},
	{m_p_transition + 66, 8, 0}, {m_p_transition + 66, -1, 5}, {m_p_transition + 71, 12, 0},
	{m_p_transition + 71, -1, 2}, {m_p_transition + 73, -1, 7}, {m_p_transition + 80, -1, 2},
	{m_p_transition + 82, -1, 3}, {m_p_transition + 85, -1, 4}, {m_p_transition + 89, -1, 5},
	{m_p_transition + 94, -1, 2}, {m_p_transition + 96, -1, 7}, {m_p_transition + 103, -1, 2},
	{m_p_transition + 105, -1, 3}, {m_p_transition + 108, -1, 4}
};

#endif // __XML_LEXER_XML10_COMPLIANT_CHARSET

CXML_Lexer::TokenEmitFunc CXML_Lexer::m_p_emit_func_list[] = {
	EmitWhitespace, EmitComment, EmitCData, EmitElem, EmitElem, EmitElem, EmitElem,
	EmitOperator, EmitOperator, EmitOperator, EmitOperator, EmitName, EmitAttValue
};

/*
 *	CXML_Lexer::CXML_Lexer(const char *p_s_filename)
 *		- opens a new file p_s_filename
 *		- call b_Status() to see wheter constructor succeeded
 */
CXML_Lexer::CXML_Lexer(const char *p_s_filename)
	:m_scanner(m_p_state, CTokenEmitAdapter(m_t_token, m_p_emit_func_list)), m_t_token(0, 0, token_EOF)
{
	__FuncGuard("CXML_Lexer::CXML_Lexer");

	if((m_p_file = new(std::nothrow) CFileReader(p_s_filename)) && !m_p_file->b_Status()) {
		delete m_p_file;
		m_p_file = 0;
	}
	// create file reader and open given file
}

/*
 *	CXML_Lexer::~CXML_Lexer()
 *		- destructor
 */
CXML_Lexer::~CXML_Lexer()
{
	__FuncGuard("CXML_Lexer::~CXML_Lexer");

	if(m_p_file)
		delete m_p_file;
}

/*
 *	bool CXML_Lexer::b_Status() const
 *		- returns status of lexer
 *		- returns true if constructor successfuly opened the file, otherwise returns false
 *		- doesn't indicate parsing errors
 */
bool CXML_Lexer::b_Status() const
{
	__FuncGuard("CXML_Lexer::b_Status");

	_ASSERTE(!m_p_file || m_p_file->b_Status());
	// if status was bad, constructor would've deleted m_p_file

	return m_p_file != 0 /*&& m_p_file->b_Status()*/;
}

/*
 *	int CXML_Lexer::n_IO_Error() const
 *		- returns CUnicodeFile error codes
 *		- always returns CUnicodeFile::error_InvalidOperation if no file was opened
 */
int CXML_Lexer::n_IO_Error() const
{
	return (m_p_file)? m_p_file->n_IO_Error() : CUnicodeFile::error_InvalidOperation;
}

/*
 *	bool CXML_Lexer::SetEncoding(int n_encoding = CUnicodeFile::code_UTF_8)
 *		- force some kind of encoding (after reading <?xml version="1.0" encoding="..."?>)
 *		- n_encoding is one of CUnicodeFile::code_ASCII, code_UTF_8 or code_UTF_16_LE
 */
bool CXML_Lexer::SetEncoding(int n_encoding)
{
	__FuncGuard("CXML_Lexer::SetEncoding");

	if(!m_p_file)
		return false;
	return m_p_file->SetEncoding(n_encoding);
}

/*
 *	inline int CXML_Lexer::n_PeekChar()
 *		- peeks at the next character in the stream
 *		- returns character code or -1 on error / EOF
 */
inline int CXML_Lexer::n_PeekChar()
{
	__FuncGuard("CXML_Lexer::n_PeekChar");

	int n_char;
	if(!m_p_file->ReadChar(n_char))
		return -1; // EOF / i/o error
	m_p_file->UnreadChar(n_char); // put it back
	return n_char;
}

/*
 *	bool CXML_Lexer::Get_NextToken()
 *		- gets next token, returns true on success, false on failure
 *		- if the file was parsed, returns true and token is set to token_EOF
 *		- the token can be accessed by t_Cur_Token()
 *		- returns true on success, false on failure
 *		- note the token constructor might set token type to error_Code or to error_Memory
 *		  if it couldn't convert input string to it's representation or if there was not
 *		  enough free memory for it (this causes this function to fail as well)
 */
bool CXML_Lexer::Get_NextToken()
{
	__FuncGuard("CXML_Lexer::Get_NextToken");

	if(!m_p_file)
		return false;
	
#ifdef __XML_LEXER_USE_EMIT_FUNCS
	m_p_this = this;
	// set 'this' pointer for token emit functions
#endif // __XML_LEXER_USE_EMIT_FUNCS

	m_t_token.n_type = token_EOF;
	// assume EOF

	if(!m_scanner.GetToken(*m_p_file))
		return false;
	// read token

	if(m_t_token.n_type != token_EOF)
		return m_t_token.n_type >= 0;
	else {
		m_t_token = TToken(m_scanner.n_Cur_Line(),
			m_scanner.n_Cur_Column(), token_EOF);
		// set proper line & col for EOF
		return true;
	}	
}

/*
 *	bool CXML_Lexer::Get_Text(std::basic_string<TCharType> &r_s_result, bool b_erase_result)
 *		- reads text data, ending with end of file or the < character
 *		- if b_erase_result is set to true, r_s_result is erased before reading anything
 *		- may contain &amp; or \#xABCD character references
 *		- returns true on success, false on failure
 *		- note t_Cur_Token() return value is unchanged by call to this function
 */
bool CXML_Lexer::Get_Text(std::basic_string<TCharType> &r_s_result, bool b_erase_result)
{
	__FuncGuard("CXML_Lexer::Get_Text");

	if(!m_p_file)
		return false;

	if(b_erase_result)
		r_s_result.erase();
	// make sure the string is empty

	int n_char;
	while(m_p_file->ReadChar(n_char)) {
		if(n_char == '<') { // everything but <
			m_p_file->UnreadChar('<');
			return true;
		}
		// handle terminating <
#ifdef __XML_LEXER_NORMALIZE_NEWLINES
		else if(n_char == '\r') { // handle newline normalizatio
			if(m_p_file->ReadChar(n_char)) { // read \n
				if(n_char != '\n') // it was not \n, it was sole \r only
					m_p_file->UnreadChar(n_char);
			}
			n_char = '\n'; // transform to newline
		}
		// newline normalization
#endif // __XML_LEXER_NORMALIZE_NEWLINES

		m_scanner.CountChar(n_char);
		// count characters

		if(AppendChar(r_s_result, n_char))
			return false;
		// append character, perform conversion if necessary
	}

	return true; // EOF
}

/*
 *	bool CXML_Lexer::Get_PIData(std::basic_string<TCharType> &r_s_result)
 *		- reads processing instruction data, ending with the ?> sequence
 *		- the ?> sequence is not contained in r_s_result,
 *		  but is parsed (there won't be token_PIEnd)
 *		- returns true on success, false on failure
 *		- note t_Cur_Token() return value is unchanged by call to this function
 */
bool CXML_Lexer::Get_PIData(std::basic_string<TCharType> &r_s_result)
{
	__FuncGuard("CXML_Lexer::Get_PIData");

	if(!m_p_file)
		return false;

	r_s_result.erase();
	// make sure the string is empty

	int n_char;
	while(m_p_file->ReadChar(n_char)) {
		m_scanner.CountChar(n_char);
		// count characters

		if(n_char == '?') {
			if(!m_p_file->ReadChar(n_char))
				return false; // unexpected EOF
			if(n_char == '>') {
				m_scanner.CountChar('>');
				return true; // ?>
			}
			m_p_file->UnreadChar(n_char);
			n_char = '?';
		} // handle terminating ?>
#ifdef __XML_LEXER_NORMALIZE_NEWLINES
		else if(n_char == '\r') {
			if(m_p_file->ReadChar(n_char)) { // read \n
				if(n_char != '\n') {
					m_p_file->UnreadChar(n_char);
					// it was not \n, it was sole \r only
				}
			}
			n_char = '\n';
			// transform to newline
		}
		// newline normalization
#endif // __XML_LEXER_NORMALIZE_NEWLINES

		if(AppendChar(r_s_result, n_char))
			return false;
		// append character, perform conversion if necessary
	}

	return false; // unexpected EOF
}

/*
 *	inline int CXML_Lexer::n_Cur_Line() const
 *		- returns current line in input file
 */
inline int CXML_Lexer::n_Cur_Line() const
{
	return m_scanner.n_Cur_Line();
}

/*
 *	inline int CXML_Lexer::n_Cur_Column() const
 *		- returns current column in input file
 */
inline int CXML_Lexer::n_Cur_Column() const
{
	return m_scanner.n_Cur_Column();
}

/*
 *	inline const CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token() const
 *		- returns value of current token
 */
inline const CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token() const
{
	return m_t_token;
}

/*
 *	inline CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token()
 *		- returns value of current token
 */
inline CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token()
{
	return m_t_token;
}

/*
 *	inline int CXML_Lexer::n_Cur_Token() const
 *		- returns type of current token
 */
inline int CXML_Lexer::n_Cur_Token() const
{
	return m_t_token.n_type;
}


#if defined(__XML_LEXER_FORMAT_UTF_32) || \
	defined(__XML_LEXER_FORMAT_RAW_16) || \
	defined(__XML_LEXER_FORMAT_RAW_8)

namespace __raw_encoding {

inline int n_UnitCharacterSize(int UNUSED(n_char))
{
	return sizeof(CXML_Lexer::TCharType);
}

inline bool PutRawChar(CXML_Lexer::TCharType *p_dest, size_t n_space, int n_character)
{
	_ASSERTE(n_space == sizeof(CXML_Lexer::TCharType));

	const CXML_Lexer::TCharType n_char_max =
		CXML_Lexer::TCharType(n_Mask(8 * sizeof(CXML_Lexer::TCharType)));
	// equivalent to 1 << 8 * sizeof(TCharType),
	// without exceeding 32 bits when TCharType is int

	*p_dest = CXML_Lexer::TCharType(n_character);

	return n_character <= n_char_max;
}

}; // ~__raw_encoding

#endif // __XML_LEXER_FORMAT_UTF_32 || __XML_LEXER_FORMAT_RAW_16 || __XML_LEXER_FORMAT_RAW_8

namespace __rcr {

template <class CCharCountIn, class CCharCountOut, class CCharEncoder>
bool ReplaceCharRefs_int(CXML_Lexer::wstring &r_s_string,
	CCharCountIn character_size, CCharCountOut encoded_size,
	CCharEncoder character_encoder)
{
	__FuncGuard("CXML_Lexer::ReplaceCharRefs_int");

	for(size_t b = 0, e = r_s_string.length(); b < e; ++ b) {
		int n_char = r_s_string[b], n_size;
		if((n_size = character_size(n_char)) > sizeof(CXML_Lexer::TCharType)) {
			b += n_size - 1; // ++ b in the for loop
			continue;
		}
		// get a (single-byte/word/dword) character from the string

		if(b + 1 < e && n_char == '&') {
			size_t n_begin = b;
			++ b;
			// remember the position of the first escape sequence character

			size_t n_end = b;
			for(int n_char; n_end < e && character_size(n_char = r_s_string[n_end]) ==
			   sizeof(CXML_Lexer::TCharType) && n_char != ';'; ++ n_end)
				;
			if(n_end == e || r_s_string[n_end] != ';')
				return false; // no semicolon found
			// find the semicolon and make sure all the characters
			// between '&' and ';' are single-byte/word/dword

			int n_reference = 0;
			if(r_s_string[b] != '#') {
				size_t n_ref_length = n_end - (n_begin + 1);
				switch(n_ref_length) {
				case 2: // lt, gt
					if(r_s_string[b + 1] == 't') {
						if(r_s_string[b] == 'l')
							n_reference = '<'; // less-than
						else if(r_s_string[b] == 'g')
							n_reference = '>'; // greater-than
					}
					break;
				case 3: // amp
					if(r_s_string[b] == 'a' && r_s_string[b + 1] == 'm' && r_s_string[b + 2] == 'p')
						n_reference = '&'; // ampersand
					break;
				case 4: // quot, apos
					if(r_s_string[b + 2] == 'o') {
						if(r_s_string[b] == 'a' && r_s_string[b + 1] == 'p' && r_s_string[b + 3] == 's')
							n_reference = '\''; // apostrophe
						else if(r_s_string[b] == 'q' && r_s_string[b + 1] == 'u' && r_s_string[b + 3] == 't')
							n_reference = '\"'; // quotation mark
					}
					break;
				}
				// parse predefined XML entites
				// f_ixme - do these references have to be low-case? // yes, they are case sensitive (xml validtor)

				if(!n_reference)
					return false;
				// fail on unknown entities (defined using <!ENTITY> or external entities)
			} else {
				if(b + 1 < n_end && r_s_string[b + 1] == 'x') { // 'x' is case sensitive
					b += 2;
					// skip '#x'

					_ASSERTE(b != n_end);
					// lexer should enforce this

					for(; b < n_end; ++ b) {
						int n_char = r_s_string[b];
						if(n_char >= '0' && n_char <= '9')
							n_reference = (n_reference << 4) | (n_char - '0');
						else if(n_char >= 'a' && n_char <= 'f')
							n_reference = (n_reference << 4) | (n_char - 'a' + 10);
						else if(n_char >= 'A' && n_char <= 'F')
							n_reference = (n_reference << 4) | (n_char - 'A' + 10);
						else
							return false; // invalid hexadecimal number
					}
					// parse hex reference

					_ASSERTE(n_reference > 0 && n_reference <= 0x10ffff);
					// make sure it's unicode
				} else {
					++ b;
					// skip '#'

					_ASSERTE(b != n_end);
					// lexer should enforce this

					for(; b < n_end; ++ b) {
						int n_char = r_s_string[b];
						if(n_char >= '0' && n_char <= '9')
							n_reference = n_reference * 10 + (n_char - '0');
							return false; // invalid hexadecimal number
					}
					// parse hex reference

					_ASSERTE(n_reference > 0 && n_reference <= 0x10ffff);
					// make sure it's unicode
				}
			}
			_ASSERTE(n_reference > 0);
			// parse the reference

			size_t n_encoded_size = encoded_size(n_reference);
			_ASSERTE(!(n_encoded_size % sizeof(CXML_Lexer::TCharType)));
			n_encoded_size /= sizeof(CXML_Lexer::TCharType);
			_ASSERTE(n_encoded_size <= (n_end + 1) - n_begin); // make sure there is enough space (should be, utf-8 is the least effective encoding in term of number of scalars (regardless of size) per encoded character, but it is still more efficient than any predefined character reference (or decimal / hexadecimal character reference))
			// determine size of the encoded character

			if(!character_encoder(&r_s_string[n_begin],
			   n_encoded_size * sizeof(CXML_Lexer::TCharType), n_reference))
				return false; // can't encode the reference
			// place the encoded reference in the string

			r_s_string.erase(r_s_string.begin() + (n_begin + n_encoded_size),
				r_s_string.begin() + (n_end + 1));
			// erase the reference, except for the first n_encoded_size charactes

			e -= (n_end + 1) - (n_begin + n_encoded_size);
			_ASSERTE(e == r_s_string.length());
			// account for the lost characters
		}
		// handle entity references
	}
	// for each char

	return true;
}

};

bool CXML_Lexer::ReplaceCharRefs(wstring &r_s_string)
{
	__FuncGuard("CXML_Lexer::ReplaceCharRefs");

#if defined(__XML_LEXER_FORMAT_UTF_32) || \
	defined(__XML_LEXER_FORMAT_RAW_16) || \
	defined(__XML_LEXER_FORMAT_RAW_8)

	return __rcr::ReplaceCharRefs_int(r_s_string, __raw_encoding::n_UnitCharacterSize,
		__raw_encoding::n_UnitCharacterSize, __raw_encoding::PutRawChar);
	// can operate on r_s_string directly (characters are bytes)

#elif defined(__XML_LEXER_FORMAT_UTF_16)

	return __rcr::ReplaceCharRefs_int(r_s_string, CUniConv::n_UTF16_Char_Size,
		CUniConv::n_UTF16_Encode_Size, CUniConv::EncodeCharacter_UTF16_LE_Strict);
	// no need to decode to utf-32, just skip high characters (escape sequence characters are all below 0x80)

#elif defined(__XML_LEXER_FORMAT_UTF_8)

	return __rcr::ReplaceCharRefs_int(r_s_string, CUniConv::n_UTF8_Char_Size,
		CUniConv::n_UTF8_Encode_Size, CUniConv::EncodeCharacter_UTF8_Strict);
	// no need to decode to utf-32, just skip high characters (escape sequence characters are all below 0x80)

#endif

	return true;
}

/*
 *	static bool CXML_Lexer::ReplaceCharRefs(int *p_s_buffer)
 *		- replaces character references such as &amp; &\#0123; or &\#xABCD;
 */
bool CXML_Lexer::ReplaceCharRefs(int *p_s_buffer)
{
	__FuncGuard("CXML_Lexer::ReplaceCharRefs");

	for(; *p_s_buffer; ++ p_s_buffer) {
		int n_search_char = *p_s_buffer;
		if(n_search_char == '&') {
			int *p_s_dest = p_s_buffer;
			for(; *p_s_buffer; ++ p_s_buffer, ++ p_s_dest) {
				int n_char = *p_s_buffer;
				if(n_char == '&') {
					if(p_s_buffer[1] != '#') {
						int *p_s_ref = ++ p_s_buffer;
						while(*p_s_buffer != ';')
							++ p_s_buffer;
						size_t n_ref_length = p_s_buffer - p_s_ref;
						// get reference and i's length

						int n_reference = 0;
						// referenced char

						switch(n_ref_length) {
						case 2: // lt, gt
							if(p_s_ref[1] == 't') {
								if(p_s_ref[0] == 'l')
									n_reference = '<'; // less-than
								else if(p_s_ref[0] == 'g')
									n_reference = '>'; // greater-than
							}
							break;
						case 3: // amp
							if(p_s_ref[0] == 'a' && p_s_ref[1] == 'm' && p_s_ref[2] == 'p')
								n_reference = '&'; // ampersand
							break;
						case 4: // apos, quot
							if(p_s_ref[2] == 'o') {
								if(p_s_ref[0] == 'a' && p_s_ref[1] == 'p' && p_s_ref[3] == 's')
									n_reference = '\''; // apostrophe
								else if(p_s_ref[0] == 'q' && p_s_ref[1] == 'u' && p_s_ref[3] == 't')
									n_reference = '\"'; // quotation mark
							}
							break;
						}
						// look trough predefined references
						// f_ixme - do these references have to be low-case? // yes, they are case sensitive (xml validtor)

						if(n_reference)
							*p_s_dest = n_reference;
						else {
							/*for(; n_ref_length; -- n_ref_length, ++ p_s_dest, ++ p_s_ref)
								*p_s_dest = *p_s_ref;*/
							// handle external references (leave them in the string)

							return false;
							// fail on external references (or on references defined
							// using <!ENTITY> which is not parsed) // f_ixme - does term external reference really mean reference defined by DTD? // yes
						}
						// output
					} else {
						p_s_buffer += 2;
						// skip the &# characters

						int n_value = 0;
						if(*p_s_buffer == 'x') { // f_ixme - does this 'x' character have to be lowcase? // yes (xml validtor)
							++ p_s_buffer;
							// skip the x character

							_ASSERTE(*p_s_buffer != ';');
							// lexer should enforce this

							for(; *p_s_buffer != ';'; ++ p_s_buffer) {
								int n_digit, n_digit_char = *p_s_buffer;
								if(n_digit_char >= '0' && n_digit_char <= '9')
									n_digit = n_digit_char - '0';
								else if(n_digit_char >= 'A' && n_digit_char <= 'F')
									n_digit = n_digit_char - 'A' + 10;
								else /*if(n_digit_char >= 'a' && n_digit_char <= 'f')*/ {
									_ASSERTE(n_digit_char >= 'a' && n_digit_char <= 'f');
									n_digit = n_digit_char - 'a' + 10;
								}
								// parse digit

								n_value = (n_value << 4) | n_digit;
								// accumulate
							}
							// parse hex character reference
						} else {
							_ASSERTE(*p_s_buffer != ';');
							// lexer should enforce this

							for(; *p_s_buffer != ';'; ++ p_s_buffer) {
								_ASSERTE(*p_s_buffer >= '0' && *p_s_buffer <= '9');
								int n_digit = *p_s_buffer - '0';
								// parse digit

								n_value = (n_value * 10) + n_digit;
								// accumulate
							}
							// parse decimal character reference
						}
						// parse character reference

						_ASSERTE(*p_s_buffer == ';');
						// character references end with ';'

						if(n_value > 0x10ffff)
							return false;
						*p_s_dest = n_value;
						// output referenced character
					}
				} else
					*p_s_dest = n_char; // copy ordinary character
			}
			// shrink the string, replace character references

			*p_s_dest = 0;
			// terminating 0

			return true;
		}
		// look for the first character reference
	}

	return true;
}

#ifdef __XML_LEXER_NORMALIZE_NEWLINES

void CXML_Lexer::NormalizeNewlines(int *p_s_buffer)
{
	__FuncGuard("CXML_Lexer::NormalizeNewlines");

	for(int n_char = *p_s_buffer; n_char; n_char = *(++ p_s_buffer)) { // simple loop
		if(n_char == '\r') {
			if(p_s_buffer[1] != '\n')
				*p_s_buffer = '\n'; // simple replace, can go on in simple loop
			else {
				*p_s_buffer = '\n'; // replace '\r' by '\n'
				int *p_s_dest = ++ p_s_buffer; // destination pointer
				++ p_s_buffer; // skip to character past \r

				for(int n_char = *p_s_buffer; n_char; n_char = *(++ p_s_buffer), ++ p_s_dest) {
					if(n_char != '\r')
						*p_s_dest = n_char;
					else {
						*p_s_dest = '\n'; // replace \r by \n
						if(p_s_buffer[1] == '\n')
							++ p_s_buffer; // skip \n following \r we just replaced
					}
				}

				*p_s_dest = 0; // terminating zero

				return;
			}
		}
	}
}

#endif // __XML_LEXER_NORMALIZE_NEWLINES

/*
 *	static bool CXML_Lexer::AppendChar(wstring &r_s_value, int n_char)
 *		- append char n_char (raw 32-bit unicode) to string r_s_value,
 *		  while applying charset conversion, given by __XML_LEXER_FORMAT_*
 *		- returns 0 on success, -1 on code conversion error or
 *		  -2 on insufficient memory error
 */
int CXML_Lexer::AppendChar(wstring &r_s_value, int n_char)
{
#if defined(__XML_LEXER_FORMAT_UTF_32) || \
	defined(__XML_LEXER_FORMAT_RAW_16) || \
	defined(__XML_LEXER_FORMAT_RAW_8)
	if(!stl_ut::Reserve_1More(r_s_value))
		return -2;
	// make sure there's enough space

	const unsigned int n_char_max =
		CXML_Lexer::TCharType(n_Mask(8 * sizeof(CXML_Lexer::TCharType)));
	// equivalent to 1 << 8 * sizeof(TCharType),
	// without exceeding 32 bits when TCharType is int

	if(unsigned(n_char) > n_char_max) {
		return -1;
		// too high character to encode
	}
	r_s_value += (TCharType)n_char;
	// convert character to raw number
#elif defined(__XML_LEXER_FORMAT_UTF_16)
	if(n_char < 0x10000) {
		/*if(n_char == 0xfffe || n_char == 0xffff || (n_char >= 0xfdd0 && n_char <= 0xfdef))
			return -1;*/
		// noncharacters

		if(!stl_ut::Reserve_1More(r_s_value))
			return -2;
		// make sure there's enough space

		r_s_value += (TCharType)n_char;
		// save as a single value
	} else if(n_char <= 0x10ffff) {
		if(!stl_ut::Reserve_NMore(r_s_value, 2))
			return -2;
		// make sure there's enough space

		int n_lead = (0xd800 - (0x10000 >> 10)) + (n_char >> 10);
		r_s_value += (TCharType)n_lead;
		int n_tail = 0xdc00 + (n_char & 0x3ff);
		r_s_value += (TCharType)n_tail;
		// save as surrogate pair
	} else {
		return -1;
		// too high character to encode
	}
#elif defined(__XML_LEXER_FORMAT_UTF_8)
	if(n_char <= 0x7f) {
		if(!stl_ut::Reserve_1More(r_s_value))
			return -2;
		// make sure there's enough space

		r_s_value += (TCharType)n_char;
		// save as a single value
	} else if(n_char <= 0x7ff) {
		if(!stl_ut::Reserve_NMore(r_s_value, 2))
			return -2;
		// make sure there's enough space

		int n_char0 = 0xc0 | (n_char >> 6);
		r_s_value += (TCharType)n_char0;
		int n_char1 = 0x80 | (n_char & 0x3f);
		r_s_value += (TCharType)n_char1;
		// save as pair of values
	} else if(n_char <= 0xffff) {
		if(n_char >= 0xd800 && n_char <= 0xdfff)
			return -1;
		// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

		if(!stl_ut::Reserve_NMore(r_s_value, 3))
			return -2;
		// make sure there's enough space

		int n_char0 = 0xe0 | (n_char >> 12);
		r_s_value += (TCharType)n_char0;
		int n_char1 = 0x80 | ((n_char >> 6) & 0x3f);
		r_s_value += (TCharType)n_char1;
		int n_char2 = 0x80 | (n_char & 0x3f);
		r_s_value += (TCharType)n_char2;
		// save as trinity of values
	} else if(n_char <= 0x10ffff) {
		if(!stl_ut::Reserve_NMore(r_s_value, 4))
			return -2;
		// make sure there's enough space

		int n_char0 = 0xf0 | (n_char >> 18);
		r_s_value += (TCharType)n_char0;
		int n_char1 = 0x80 | ((n_char >> 12) & 0x3f);
		r_s_value += (TCharType)n_char1;
		int n_char2 = 0x80 | ((n_char >> 6) & 0x3f);
		r_s_value += (TCharType)n_char2;
		int n_char3 = 0x80 | (n_char & 0x3f);
		r_s_value += (TCharType)n_char3;
		// save as quadruple of values
	} else {
		return -1;
		// too high character to encode
	}
#endif
	return 0;
}

bool CXML_Lexer::EmitWhitespace(const int *p_s_buffer, int UNUSED(n_regexp_id), int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitWhitespace");

	_ASSERTE(n_regexp_id == 0);

#ifdef __XML_LEXER_NORMALIZE_NEWLINES
	CXML_Lexer::NormalizeNewlines((int*)p_s_buffer);
	// t_odo - transform \r\n and \r to \n
#endif // __XML_LEXER_NORMALIZE_NEWLINES

	*p_out_token = TToken(n_line, n_column, token_White, p_s_buffer);
	return true;
}

bool CXML_Lexer::EmitComment(const int *p_s_buffer, int UNUSED(n_regexp_id), int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitComment");

	_ASSERTE(n_regexp_id == 1);

#ifdef __XML_LEXER_NORMALIZE_NEWLINES
	CXML_Lexer::NormalizeNewlines((int*)p_s_buffer);
	// t_odo - transform \r\n and \r to \n
#endif // __XML_LEXER_NORMALIZE_NEWLINES

	_ASSERTE(CWideUtils::strlen_w(p_s_buffer) >= 7); // <!-- -->

	*p_out_token = TToken(n_line, n_column, token_Comment,
		p_s_buffer + 4, CWideUtils::strlen_w(p_s_buffer) - 7);
	return true;
}

bool CXML_Lexer::EmitCData(const int *p_s_buffer, int UNUSED(n_regexp_id), int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitCData");

	_ASSERTE(n_regexp_id == 2);

	_ASSERTE(CWideUtils::strlen_w(p_s_buffer) >= 12); // <![CDATA[ ]]>
	*p_out_token = TToken(n_line, n_column, token_CData,
		p_s_buffer + 9, CWideUtils::strlen_w(p_s_buffer) - 12);
	return p_out_token->n_type >= 0;
}

bool CXML_Lexer::EmitElem(const int *UNUSED(p_s_buffer), int n_regexp_id, int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitElem");

	_ASSERTE(n_regexp_id >= 3 && n_regexp_id <= 6);

	*p_out_token = TToken(n_line, n_column, token_Elem + n_regexp_id - 3); // < </ <? <!
	return p_out_token->n_type >= 0;
}

bool CXML_Lexer::EmitOperator(const int *UNUSED(p_s_buffer), int n_regexp_id, int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitOperator");

	_ASSERTE(n_regexp_id >= 7 && n_regexp_id <= 10); // > /> ?> =

	*p_out_token = TToken(n_line, n_column, token_End + n_regexp_id - 7);
	return true;
}

bool CXML_Lexer::EmitName(const int *p_s_buffer, int UNUSED(n_regexp_id), int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitName");

	_ASSERTE(n_regexp_id == 11);

	*p_out_token = TToken(n_line, n_column, token_Name, p_s_buffer);
	return p_out_token->n_type >= 0;
}

bool CXML_Lexer::EmitAttValue(const int *p_s_buffer, int UNUSED(n_regexp_id), int n_line, int n_column, TToken *p_out_token)
{
	__FuncGuard("CXML_Lexer::EmitAttValue");

	_ASSERTE(n_regexp_id == 12);

	if(!ReplaceCharRefs((int*)p_s_buffer + 1)) {
		p_out_token->n_type = error_CharRef;
		return false;
	}

	_ASSERTE(CWideUtils::strlen_w(p_s_buffer) >= 2); // '[^']*' | "[^"]*"
	*p_out_token = TToken(n_line, n_column, token_AttValue,
		p_s_buffer + 1, CWideUtils::strlen_w(p_s_buffer) - 2);
	return p_out_token->n_type >= 0;
}

/*
 *								=== ~CXML_Lexer ===
 */

/*
 *								=== CXML_EventParser ===
 */

/*
 *	CXML_EventParser::CXML_EventParser(const char *p_s_filename)
 *		- constructor; creates parser and opens file p_s_filename
 *		- it's necessary to call Read() to read the first (and any following) node
 */
CXML_EventParser::CXML_EventParser(const char *p_s_filename)
	:m_lexer(p_s_filename), m_n_node_type(node_None), m_n_state(state_NoError)
{
	__FuncGuard("CXML_EventParser::CXML_EventParser");

	if(!m_lexer.b_Status())
		m_n_state = state_LexerInitError;
}

/*
 *	int CXML_EventParser::n_State() const
 *		- returns error state of parser (but does not clear it)
 *		- if no error occured, returns state_NoError
 */
int CXML_EventParser::n_State() const
{
	return m_n_state;
}

/*
 *	const char *CXML_EventParser::p_s_State() const
 *		- returns string description of parser state
 */
const char *CXML_EventParser::p_s_State() const
{
	return CXML_EventParser::p_s_ErrorString(m_n_state);
}

/*
 *	int CXML_EventParser::n_Cur_Line() const
 *		- returns current line in input file
 */
int CXML_EventParser::n_Cur_Line() const
{
	return m_lexer.n_Cur_Line();
}

/*
 *	int CXML_EventParser::n_Cur_Column() const
 *		- returns current column in input file
 */
int CXML_EventParser::n_Cur_Column() const
{
	return m_lexer.n_Cur_Column();
}

/*
 *	bool CXML_EventParser::Read()
 *		- reads an xml node
 *		- use n_NodeType() to determine what kind of node was just read
 *		- returns true on success and false on EOF / failure
 *		  (use n_State() to decide between EOF and failure)
 *		- note this always fails if parser is in error state (so it's not
 *		  necessary to call n_State() unless Read() fails)
 */
bool CXML_EventParser::Read()
{
	__FuncGuard("CXML_EventParser::Read");

	if(m_n_state != state_NoError)
		return false;
	// won't parse in error state

	m_attrib_list.clear();
	m_s_name.erase();
	m_s_value.erase();
	// clear params

	int n_char;
	if((n_char = m_lexer.n_PeekChar()) < 0) {
		if(m_lexer.n_IO_Error() != CUnicodeFile::error_NoError)
			m_n_state = state_LexerError; // lexer (i/o?) error
#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		if(!m_node_stack.empty()) {
			m_n_state = state_MalformedDoc;
			return false;
		}
		// all elements must be ended upon eof
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		return false; // EOF
	}
	// peek character

	if(n_char == '<' || n_char == ' ' || n_char == '\t' || n_char == '\r' || n_char == '\n') {
		if(!m_lexer.Get_NextToken()) {
			m_n_state = n_GetTokenErrorType();
			return false;
		}
		// get next token

		switch(m_lexer.n_Cur_Token()) {
		case CXML_Lexer::token_EOF:
#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			if(!m_node_stack.empty()) {
				m_n_state = state_MalformedDoc;
				return false;
			}
			// all elements must be ended upon eof
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			return false; // end of file (do not raise error flag)
		case CXML_Lexer::error_Memory:
			m_n_state = state_NoMemory;
			return false; // not enough memory
		case CXML_Lexer::error_Code: 
			m_n_state = state_CodeError;
			return false; // code conversion error (unicode to CXML_Lexer::TCharType)
		case CXML_Lexer::error_CharRef:
			m_n_state = state_MalformedDoc;
			return false; // attribute values must not contain external character references
		case CXML_Lexer::token_Elem:
		case CXML_Lexer::token_EndElem:
		case CXML_Lexer::token_PIElem:
		case CXML_Lexer::token_DTDElem:
			return Parse_Element();
		case CXML_Lexer::token_Comment:
			m_s_value.swap(m_lexer.t_Cur_Token().s_value);
			m_n_node_type = node_Comment;
			return true;
		case CXML_Lexer::token_CData:
			m_s_value.swap(m_lexer.t_Cur_Token().s_value);
			m_n_node_type = node_CData;
			return true;
		case CXML_Lexer::token_White:
			m_s_value.swap(m_lexer.t_Cur_Token().s_value);
			m_n_node_type = node_Whitespace;
			// assume we really have whitespace node

			{
				int n_char2;
				if((n_char2 = m_lexer.n_PeekChar()) < 0) {
					if(m_lexer.n_IO_Error() != CUnicodeFile::error_NoError) {
						m_n_state = state_LexerError; // lexer (i/o?) error
						return false;
					}

					return true; // EOF (but we've already read whitespace)
				}
				// peek one more character

				if(n_char2 != '<') {
					m_n_node_type = node_Text;
					if(!m_lexer.Get_Text(m_s_value, false)) {
						m_n_state = n_GetTokenErrorType();
						return false;
					}
					return true;
					// text node
				}
				// what follows is text, not markup. the whitespace is part of this text.
			}
			// concatenate white and possible following text

			return true;
		default:
			_ASSERTE(m_lexer.n_Cur_Token() != CXML_Lexer::token_EOF);
			m_n_state = state_UnexpectedToken;
			return false;
		}
	} /*else*/ {
		m_n_node_type = node_Text;
		if(!m_lexer.Get_Text(m_s_value, true)) {
			m_n_state = n_GetTokenErrorType();
			return false;
		}
		return true;
		// text node
	}
}

/*
 *	bool CXML_EventParser::Parse_Element()
 *		- parses element attributes, returns true on success, false on failure
 *		- raises error state on failure
 */
bool CXML_EventParser::Parse_Attribs()
{
	for(;;) {
		do {
			if(!m_lexer.Get_NextToken()) {
				m_n_state = n_GetTokenErrorType();
				return false;
			}
		} while(m_lexer.n_Cur_Token() == CXML_Lexer::token_White);
		if(m_lexer.n_Cur_Token() == CXML_Lexer::token_End) {
			if(m_n_node_type != node_Element &&
			   m_n_node_type != node_EndElement) {
				m_n_state = state_UnexpectedToken;
				return false;
			}
#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			if(m_n_node_type == node_Element) {
				if(!stl_ut::Reserve_1More(m_node_stack)) {
					m_n_state = state_NoMemory;
					return false;
				}
				m_node_stack.push_back(m_s_name);
			}
			// copy node name (only in case it's node_Element, not node_SingleElement)
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			break;
		} else if(m_lexer.n_Cur_Token() == CXML_Lexer::token_PIEnd) {
			if(m_n_node_type != node_XmlDecl &&
			   m_n_node_type != node_ProcInstr) {
				m_n_state = state_UnexpectedToken;
				return false;
			}
			break;
		} else if(m_lexer.n_Cur_Token() == CXML_Lexer::token_SingleEnd) {
			if(m_n_node_type != node_Element) {
				m_n_state = n_GetUnexpectedErrorType();
				return false;
			}
			m_n_node_type = node_SingleElement;
			break;
		}
		// get next token (different from > or />)

		if(m_lexer.n_Cur_Token() != CXML_Lexer::token_Name) {
			m_n_state = n_GetUnexpectedErrorType();
			return false;
		}

		TAttrib attrib;
		attrib.s_name.swap(m_lexer.t_Cur_Token().s_value);
		// get attrib name

#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		if(std::find(m_attrib_list.begin(), m_attrib_list.end(),
		   attrib.s_name) != m_attrib_list.end()) {
			m_n_state = state_MalformedDoc;
			return false;
		}
		// well-formedness constraint: no two attribs with the same name
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

		for(int n_pass = 0; n_pass < 2; ++ n_pass) {
			do {
				if(!m_lexer.Get_NextToken()) {
					m_n_state = n_GetTokenErrorType();
					return false;
				}
			} while(m_lexer.n_Cur_Token() == CXML_Lexer::token_White);
			if(m_lexer.n_Cur_Token() != ((n_pass)?
			   CXML_Lexer::token_AttValue : CXML_Lexer::token_Assign)) {
				m_n_state = n_GetUnexpectedErrorType();
				return false;
			}
		}
		// get next two tokens (assing and attribute value)

		attrib.s_value.swap(m_lexer.t_Cur_Token().s_value);
		// get attrib value

#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		if(attrib.s_value.find('<') != std::string::npos) {
			m_n_state = state_MalformedDoc;
			return false;
		}
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		// well-formedness constraint: no attrib values containing < or external entity references
		// having < in attrib value doesn't pass trough lexer, but there still can be &lt;

		if(!ReserveAttrib())
			return false;
		m_attrib_list.push_back(attrib);
		// add attribute to the list
	}
	// parse attributes

	return true;
}

/*
 *	bool CXML_EventParser::Parse_Element()
 *		- parses element node, returns true on success, false on failure
 *		- raises error state on failure
 */
bool CXML_EventParser::Parse_Element()
{
	int n_token_type = m_lexer.n_Cur_Token();
	// remember token type (element type)

	if(!m_lexer.Get_NextToken()) {
		m_n_state = n_GetTokenErrorType();
		return false;
	}
	if(m_lexer.n_Cur_Token() != CXML_Lexer::token_Name) {
		m_n_state = n_GetUnexpectedErrorType();
		return false;
	}
	// parse name of element

	m_s_name.swap(m_lexer.t_Cur_Token().s_value);
	// swap name (dammaging current token t_Cur_Token())

	if(n_token_type == CXML_Lexer::token_PIElem) {
		if(m_s_name.length() == 3 &&
		   (m_s_name[0] == 'x' || m_s_name[0] == 'X') &&
		   (m_s_name[1] == 'm' || m_s_name[1] == 'M') &&
		   (m_s_name[2] == 'l' || m_s_name[2] == 'L')) {
#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			if(m_n_node_type != node_None) {
				m_n_state = state_MalformedDoc;
				return false;
			}
			// well-formedness constraint: must be first element in the file
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

			m_n_node_type = node_XmlDecl;
			// continue to parse attributes

			if(!Parse_Attribs())
				return false;
			// parse attributes

			if(!Detect_Set_Encoding()) {
				m_n_state = state_BadEncoding;
				return false;
			}
			// try to set appropriate encoding if current node is node_XmlDecl

			return true;
		} else
			return Parse_ProcInstr();
	} else if(n_token_type == CXML_Lexer::token_DTDElem)
		return Parse_DTD();
	else if(n_token_type == CXML_Lexer::token_EndElem) {
		m_n_node_type = node_EndElement;
		do {
			if(!m_lexer.Get_NextToken()) {
				m_n_state = n_GetTokenErrorType();
				return false;
			}
		} while(m_lexer.n_Cur_Token() == CXML_Lexer::token_White);
		if(m_lexer.n_Cur_Token() != CXML_Lexer::token_End) {
			m_n_state = n_GetUnexpectedErrorType();
			return false;
		}

#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
		if(m_node_stack.back() != m_s_name) {
			m_n_state = state_MalformedDoc;
			return false;
		}
		m_node_stack.erase(m_node_stack.end() - 1);
		// watch out for bad element pairing and nesting
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

		return true;
		// end element is simple: </name S? >
	} else /*if(n_token_type == CXML_Lexer::token_Elem)*/ {
		_ASSERTE(n_token_type == CXML_Lexer::token_Elem);
		m_n_node_type = node_Element;

		if(!Parse_Attribs())
			return false;
		// parse attributes

		return true;
	}
	// determine node type, handle special cases
}

/*
 *	bool CXML_EventParser::Parse_ProcInstr()
 *		- parses PI node, returns true on success, false on failure
 *		- raises error state on failure
 */
bool CXML_EventParser::Parse_ProcInstr()
{
	m_n_node_type = node_ProcInstr;

	if(!m_lexer.Get_NextToken()) {
		m_n_state = n_GetTokenErrorType();
		return false;
	}
	if(m_lexer.n_Cur_Token() == CXML_Lexer::token_White) {
		if(!m_lexer.Get_PIData(m_s_value)) {
			m_n_state = n_GetTokenErrorType();
			return false; // PI contains some more blah
		}
		return true;
	} else if(m_lexer.n_Cur_Token() == CXML_Lexer::token_PIEnd) {
		_ASSERTE(m_s_value.empty());
		return true; // empty PI, m_s_value was cleared at the beginning
	} else {
		m_n_state = n_GetUnexpectedErrorType();
		return false; // nothing else is allowed inside PI
	}
	// PI doesn't contain attributes in common format
}

/*
 *	bool CXML_EventParser::Parse_DTD()
 *		- parses DTD node, returns true on success, false on failure
 *		- raises error state on failure
 */
bool CXML_EventParser::Parse_DTD()
{
	m_n_node_type = node_DocTypeDecl;

#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
	if(!m_node_stack.empty()) {
		m_n_state = state_MalformedDoc;
		return false;
	}
	// well-formedness constraint: must be root element
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

	if(!CWideUtils::strmatch_w(m_s_name, "DOCTYPE")) {
		m_n_state = state_UnsupportedNode;
		return false;
	}
	// external DTD's are supported only

	// <!DOCTYPE greeting (( SYSTEM "hello.dtd" ) |
	//					   ( PUBLIC "pubid" "blah.dtd" ))? ( [ xxx ] )? >

	for(int n_pass = 0; n_pass < 2; ++ n_pass) {
		if(!m_lexer.Get_NextToken()) {
			m_n_state = n_GetTokenErrorType();
			return false;
		}
		if(m_lexer.n_Cur_Token() != ((n_pass)?
		   CXML_Lexer::token_Name : CXML_Lexer::token_White)) {
			m_n_state = n_GetUnexpectedErrorType();
			return false;
		}
	}
	// get DTD name (mandatory)

	if(!ReserveAttrib())
		return false;
	TAttrib attrib;
	attrib.s_name.swap(m_lexer.t_Cur_Token().s_value);
	m_attrib_list.push_back(attrib);
	// add attrib with name only

	for(;;) {
		int n_char;
		if((n_char = m_lexer.n_PeekChar()) < 0) {
			if(m_lexer.n_IO_Error() != CUnicodeFile::error_NoError) {
				m_n_state = state_LexerError; // lexer (i/o?) error
				return false;
			}
			break;
		}
		if(n_char == '[') {
			m_n_state = state_UnsupportedNode;
			// this looks like internal subset, that's not supported right now
			return false;
		}
		// peek character

		if(!m_lexer.Get_NextToken()) {
			m_n_state = n_GetTokenErrorType();
			return false;
		}
		int n_token;
		if((n_token = m_lexer.n_Cur_Token()) == CXML_Lexer::token_White)
			continue;
		else if(n_token == CXML_Lexer::token_End)
			break;
		else if(n_token == CXML_Lexer::token_Name) {
			int n_value_num = 0;
			if(CWideUtils::strmatch_w(m_lexer.t_Cur_Token().s_value, "SYSTEM"))
				n_value_num = 1;
			else if(CWideUtils::strmatch_w(m_lexer.t_Cur_Token().s_value, "PUBLIC"))
				n_value_num = 2;
			else {
				m_n_state = state_UnexpectedToken;
				return false;
			}
			// system / public subset

			if(!ReserveAttrib())
				return false;
			TAttrib attrib;
			attrib.s_name.swap(m_lexer.t_Cur_Token().s_value);
			m_attrib_list.push_back(attrib);
			// add attrib with name only

			for(int i = 0; i < n_value_num; ++ i) {
				for(int n_pass = 0; n_pass < 2; ++ n_pass) {
					if(!m_lexer.Get_NextToken()) {
						m_n_state = n_GetTokenErrorType();
						return false;
					}
					if(m_lexer.n_Cur_Token() != ((n_pass)?
					   CXML_Lexer::token_AttValue : CXML_Lexer::token_White)) {
						m_n_state = n_GetUnexpectedErrorType();
						return false;
					}
				}
				// get attrib value

				if(!ReserveAttrib())
					return false;
				TAttrib attrib;
				attrib.s_value.swap(m_lexer.t_Cur_Token().s_value);
				m_attrib_list.push_back(attrib);
				// add attrib with value only
			}
			// parse attrib values
		} else {
			m_n_state = n_GetUnexpectedErrorType();
			return false;
		}
	}

	return true;
}

/*
 *	bool CXML_EventParser::ReserveAttrib()
 *		- reserves space for one more attribute in m_attrib_list (if necessary)
 *		- returns true on success, false on failure
 *		- sets m_n_state to state_NoMemory on failure
 */
bool CXML_EventParser::ReserveAttrib()
{
	if(!stl_ut::Reserve_1More(m_attrib_list)) {
		m_n_state = state_NoMemory;
		return false;
	}
	return true;
}

/*
 *	int CXML_EventParser::n_NodeType() const
 *		- returns type of last parsed node
 *		- returns node_None if Read() has not been called yet
 *		- note the return value is invalid if parser is in error state
 */
int CXML_EventParser::n_NodeType() const
{
	return m_n_node_type;
}

/*
 *	const CXML_EventParser::wstring &CXML_EventParser::s_Name() const
 *		- returns name of last parsed node (only node_XmlDecl, node_DocTypeDecl,
 *		  node_ProcInstr, node_Element and node_SingleElement have names)
 */
const CXML_EventParser::wstring &CXML_EventParser::s_Name() const
{
	return m_s_name;
}

/*
 *	CXML_EventParser::wstring &CXML_EventParser::s_Name()
 *		- returns name of last parsed node (only node_XmlDecl, node_DocTypeDecl,
 *		  node_ProcInstr, node_Element and node_SingleElement have names)
 */
CXML_EventParser::wstring &CXML_EventParser::s_Name()
{
	return m_s_name;
}

/*
 *	const CXML_EventParser::wstring &CXML_EventParser::s_Value() const
 *		- returns value of last parsed node (only node_ProcInstr, node_CData,
 *		  node_Text, node_Comment and node_Whitespace have values)
 *		- note node_Text may contain character references such as &amp; or \#xABCD;
 *		  which haven't been replaced (there might be external character references)
 */
const CXML_EventParser::wstring &CXML_EventParser::s_Value() const
{
	return m_s_value;
}

/*
 *	CXML_EventParser::wstring &CXML_EventParser::s_Value()
 *		- returns value of last parsed node (only node_ProcInstr, node_CData,
 *		  node_Text, node_Comment and node_Whitespace have values)
 *		- note node_Text may contain character references such as &amp; or \#xABCD;
 *		  which haven't been replaced (there might be external character references)
 */
CXML_EventParser::wstring &CXML_EventParser::s_Value()
{
	return m_s_value;
}

/*
 *	const std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List() const
 *		- returns const reference to attribute list of last parsed node
 */
const std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List() const
{
	return m_attrib_list;
}

/*
 *	std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List()
 *		- returns reference to attribute list of last parsed node
 */
std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List()
{
	return m_attrib_list;
}

/*
 *	int CXML_EventParser::n_Attribute_Num() const
 *		- returns number of node attributes (only node_XmlDecl, node_Element and
 *		  node_SingleElement may have parameters)
 *		- note node_ProcInstr may have parameters as well, but those are not parsed and
 *		  are all returned as string by s_Value()
 *		- note mode_DocTypeDecl may have parameters, but it's not spported at the moment
 */
size_t CXML_EventParser::n_Attribute_Num() const
{
	return m_attrib_list.size();
}

/*
 *	const CXML_EventParser::TAttrib &CXML_EventParser::t_Attribute(int n_index) const
 *		- returns attribute (name and value) with zero-based index n_index
 *		- only node_XmlDecl, node_Element and node_SingleElement may have parameters
 *		- note node_ProcInstr may have parameters as well, but those are not parsed and
 *		  are all returned as string by s_Value()
 *		- note mode_DocTypeDecl may have parameters, but it's not spported at the moment
 *		- note attribute value may contain internal character references which
 *		  are automatically replaced by characters they represent
 */
const CXML_EventParser::TAttrib &CXML_EventParser::t_Attribute(size_t n_index) const
{
	return m_attrib_list[n_index];
}

/*
 *	CXML_EventParser::TAttrib &CXML_EventParser::t_Attribute(int n_index)
 *		- returns attribute (name and value) with zero-based index n_index
 *		- only node_XmlDecl, node_Element and node_SingleElement may have parameters
 *		- note node_ProcInstr may have parameters as well, but those are not parsed and
 *		  are all returned as string by s_Value()
 *		- note mode_DocTypeDecl may have parameters, but it's not spported at the moment
 *		- note attribute value may contain internal character references which
 *		  are automatically replaced by characters they represent
 */
CXML_EventParser::TAttrib &CXML_EventParser::t_Attribute(size_t n_index)
{
	return m_attrib_list[n_index];
}

/*
 *	const CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(
 *		const char *p_s_name) const
 *		- returns attribute with name equal to p_s_name or 0 if there is no such attribute
 */
const CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(const char *p_s_name) const
{
	for(size_t i = 0, n = n_Attribute_Num(); i < n; ++ i) {
#ifdef __XML_CASE_SENSITIVE_ATTRIBS
		if(CWideUtils::strmatch_w(t_Attribute(i).s_name, p_s_name))
#else // __XML_CASE_SENSITIVE_ATTRIBS
		if(CWideUtils::strimatch_w(t_Attribute(i).s_name, p_s_name))
#endif // __XML_CASE_SENSITIVE_ATTRIBS
			return &t_Attribute(i);
	}
	return 0;
}

/*
 *	CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(const char *p_s_name)
 *		- returns attribute with name equal to p_s_name or 0 if there is no such attribute
 */
CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(const char *p_s_name)
{
	for(size_t i = 0, n = n_Attribute_Num(); i < n; ++ i) {
#ifdef __XML_CASE_SENSITIVE_ATTRIBS
		if(CWideUtils::strmatch_w(t_Attribute(i).s_name, p_s_name))
#else // __XML_CASE_SENSITIVE_ATTRIBS
		if(CWideUtils::strimatch_w(t_Attribute(i).s_name, p_s_name))
#endif // __XML_CASE_SENSITIVE_ATTRIBS
			return &t_Attribute(i);
	}
	return 0;
}

/*
 *	static const char *CXML_EventParser::p_s_ErrorString(int n_state)
 *		- returns string representation of parser state n_state
 */
const char *CXML_EventParser::p_s_ErrorString(int n_state)
{
	const char *p_s_error_string_list[] = {
		"no error",
		"not enough memory",
		"can't set specified encoding",
		"code conversion error",
		"failed to open input file",
		"error reading input file",
		"lexical error",
		"unexpected EOF",
		"unexpected token",
		"well-formedness constraint broken",
		"xml node not supported"
	};

	return p_s_error_string_list[n_state];
}

/*
 *	int CXML_EventParser::n_GetUnexpectedErrorType()
 *		- decides between unexpected token and unexpected EOF
 *		- returns state_UnexpectedEOF or state_UnexpectedToken
 */
int CXML_EventParser::n_GetUnexpectedErrorType()
{
	return (m_lexer.n_Cur_Token() == CXML_Lexer::token_EOF)?
		state_UnexpectedEOF : state_UnexpectedToken;
}

/*
 *	int CXML_EventParser::n_GetTokenErrorType()
 *		- decides between lexer error and lexical error
 *		- returns state_LexerError or state_LexicalError
 */
int CXML_EventParser::n_GetTokenErrorType()
{
	return (m_lexer.n_IO_Error() == CUnicodeFile::error_NoError)?
		state_LexicalError : state_LexerError;
}

/*
 *	bool CXML_EventParser::Detect_Set_Encoding()
 *		- detects encoding specified by current node which must be node_XmlDecl
 *		  and sets it in unicode reader in lexical analyzer
 *		- returns true on success (no encoding present or encoding successfuly set) or
 *		  false on failure (specified encoding is not supported or encoding doesn't match
 *		  BOM in the xml file)
 */
bool CXML_EventParser::Detect_Set_Encoding()
{
	__FuncGuard("CXML_EventParser::Detect_Set_Encoding");

	_ASSERTE(m_n_node_type == node_XmlDecl);

	std::vector<TAttrib>::const_iterator p_attrib_it =
		std::find_if(m_attrib_list.begin(), m_attrib_list.end(), b_IsEncodingAttrib);
	if(p_attrib_it != m_attrib_list.end()) {
		const wstring &s_encoding = (*p_attrib_it).s_value;
		// should be one of:
		// "UTF-8", "UTF-16", "ISO-10646-UCS-2", "ISO-10646-UCS-4" for unicode
		// "ISO-8859-1", "ISO-8859-2", ... "ISO-8859-n" for ascii
		// "ISO-2022-JP", "Shift_JIS", and "EUC-JP" are some strange eastern encodings
		// "win-1234" or "windows-1234" for ascii

		if(s_encoding.length() > 4 && s_encoding[3] == '-') {
			if((s_encoding[0] == 'u' || s_encoding[0] == 'U') &&
			   (s_encoding[1] == 't' || s_encoding[1] == 'T') &&
			   (s_encoding[2] == 'f' || s_encoding[2] == 'F')) {
				if(s_encoding.length() == 5 && s_encoding[4] == '8')
					return m_lexer.SetEncoding(CUnicodeFile::code_UTF_8);
				else if(s_encoding.length() == 6 && s_encoding[4] == '1' && s_encoding[5] == '6')
					return m_lexer.SetEncoding(CUnicodeFile::code_UTF_16_LE);
				else
					return false; // unknown UTF
				// it's some kind of UTF encoding
			} else if((s_encoding[0] == 'i' || s_encoding[0] == 'I') &&
			   (s_encoding[1] == 's' || s_encoding[1] == 'S') &&
			   (s_encoding[2] == 'o' || s_encoding[2] == 'O')) {
				// it's some kind of ISO encoding

				if(s_encoding.length() < 10)
					return false;
				// at least 10 chars

				if(s_encoding[4] != '8' || s_encoding[5] != '8' ||
				   s_encoding[6] != '5' || s_encoding[7] != '9' ||
				   s_encoding[8] != '-' || !isdigit(s_encoding[9]))
					return false;
				// accept ISO-8859-[0-9].*

				return m_lexer.SetEncoding(CUnicodeFile::code_ASCII);
				// set encoding
#ifdef __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS
			} else if((s_encoding[0] == 'w' || s_encoding[0] == 'W') &&
			   (s_encoding[1] == 'i' || s_encoding[1] == 'I') &&
			   (s_encoding[2] == 'n' || s_encoding[2] == 'N')) {
				// it's some kind of windows proprietary encoding

				if(s_encoding.length() != 8 ||
				   !isdigit(s_encoding[7]) ||
				   !isdigit(s_encoding[5]) ||
				   !isdigit(s_encoding[6]) ||
				   !isdigit(s_encoding[7]))
					return false;
				// accept win-[0-9]{4}

				return m_lexer.SetEncoding(CUnicodeFile::code_ASCII);
				// set encoding
#endif // __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS
			} else
				return false; // unsupported encoding
#ifdef __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS
		} else if(s_encoding.length() > 8 && s_encoding[7] == '-') {
			if((s_encoding[0] == 'w' || s_encoding[0] == 'W') &&
			   (s_encoding[1] == 'i' || s_encoding[1] == 'I') &&
			   (s_encoding[2] == 'n' || s_encoding[2] == 'N') &&
			   (s_encoding[3] == 'd' || s_encoding[3] == 'D') &&
			   (s_encoding[4] == 'o' || s_encoding[4] == 'O') &&
			   (s_encoding[5] == 'w' || s_encoding[5] == 'W') &&
			   (s_encoding[6] == 's' || s_encoding[6] == 'S') &&
			   (s_encoding[7] == '-' || s_encoding[7] == '-') &&
			   isdigit(s_encoding[8]) &&
			   isdigit(s_encoding[9]) &&
			   isdigit(s_encoding[10]) &&
			   isdigit(s_encoding[11])) {
				// accept windows-[0-9]{4}

				return m_lexer.SetEncoding(CUnicodeFile::code_ASCII);
				// set encoding
			} else
				return false; // unsupported encoding
#endif // __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS
		} else {
#ifdef __XML_PARSER_IGNORE_UNKNOWN_ENCODINGS
			return m_lexer.SetEncoding(CUnicodeFile::code_ASCII); // did not set encoding, but that's ok as long as the encoding is 8-bit encoding
#else // __XML_PARSER_IGNORE_UNKNOWN_ENCODINGS
			return false; // unsupported encoding
#endif // __XML_PARSER_IGNORE_UNKNOWN_ENCODINGS
		}
		// detect and set encoding
	}

	return true; // did not set encoding, but that's ok
}

/*
 *	static inline bool CXML_EventParser::b_IsEncodingAttrib(const TAttrib &r_attrib)
 *		- predicate telling wheter name of attribute r_attrib is "encoding"
 *		- returns true if r_attrib is the encoding attribute, otherwise returns false
 */
inline bool CXML_EventParser::b_IsEncodingAttrib(const TAttrib &r_attrib)
{
#ifdef __XML_CASE_SENSITIVE_ATTRIBS
	return CWideUtils::strmatch_w(r_attrib.s_name, "encoding");
#else // __XML_CASE_SENSITIVE_ATTRIBS
	return CWideUtils::strimatch_w(r_attrib.s_name, "encoding");
#endif // __XML_CASE_SENSITIVE_ATTRIBS
	/*const wstring &r_s_name = r_attrib.s_name;
	if(r_s_name.length() != 8)
		return false;
	const char *p_s_encoding = "encoding";
	for(int i = 0; i < 8; ++ i, ++ p_s_encoding) {
		if(tolower(r_s_name[i]) != *p_s_encoding)
			return false;
	}
	return true;*/
}

/*
 *								=== ~CXML_EventParser ===
 */

/*
 *								=== CXML_FlatItemList_Parser ===
 */

/*
 *	CXML_FlatItemList_Parser::CXML_FlatItemList_Parser(const char **p_container_path,
 *		int n_container_path_length, const char **p_property_name_list, int n_property_num)
 *		- constructor; initializes flat item list parser
 *		- p_container_path is list of n_container_path_length names of nested
 *		  elements containing the element property values nest inside
 *		- p_property_name_list is list of n_property_num property names
 *		- for the following xml structure:
 *
 *		<?xml version="1.0"?>
 *		<document>
 *			<container>
 *				<prop-a>value-a</prop-a>
 *				<prop-b>value-b</prop-b>
 *				<prop-c>value-c</prop-c>
 *			</container>
 *		</document>
 *
 *		  example initialization would be:
 *
 *		const char *p_container_path[] = {"document", "container"};
 *		int n_container_path_length = 2;
 *		const char *p_property_name_list[] = {"prop-a", "prop-b", "prop-c"};
 *		int n_property_num = 3;
 *
 *		- note the container section (and in fact the document section
 *		  as well can repeat infinitely)
 *		- p_container_path and p_property_name_list are not copied, they are referenced
 *		  only and must remain in memory for parser lifetime
 */
CXML_FlatItemList_Parser::CXML_FlatItemList_Parser(const char **p_container_path, int n_container_path_length,
	const char **p_property_name_list, int n_property_num)
	:m_b_dealloc_table(true), m_p_prop_value(0), m_p_prop_specified(0)
{
	m_t_table.n_rule_num = n_container_path_length + n_property_num;
	if((m_t_table.p_rule = new(std::nothrow) TDrivingRule[m_t_table.n_rule_num])) {
		for(int i = 0; i < n_container_path_length; ++ i) {
			m_t_table.p_rule[i].p_s_name = p_container_path[i];
			m_t_table.p_rule[i].n_parent_state = i;
			m_t_table.p_rule[i].n_state = i + 1;
		}
		for(int i = 0; i < n_property_num; ++ i) {
			int j = i + n_container_path_length;
			m_t_table.p_rule[j].p_s_name = p_property_name_list[i];
			m_t_table.p_rule[j].n_parent_state = n_container_path_length;
			m_t_table.p_rule[j].n_state = j + 1;
		}
		m_t_table.n_container_state = n_container_path_length;
		m_t_table.n_property_state = n_container_path_length + 1;
		m_t_table.n_property_num = n_property_num;
	}
	// construct driving table

	m_p_prop_value = new(std::nothrow) wstring[n_property_num];
	m_p_prop_specified = new(std::nothrow) bool[n_property_num];
	// alloc property containers
}

/*
 *	CXML_FlatItemList_Parser::CXML_FlatItemList_Parser(const TDrivingTable &r_t_table)
 *		- constructor; initializes flat item list parser
 *		- r_t_table is driving table (it is referenced, not copied and thus it
 *		  must remain allocated for parser lifetime)
 *		- this is a little bit harder to use, but more efficient and of greater power as well
 *		- for the following xml structure:
 *
 *		<?xml version="1.0"?>
 *		<document>
 *			<container>
 *				<prop-a>value-a</prop-a>
 *				<prop-b>value-b</prop-b>
 *				<sub-container>
 *					<prop-c>value-c</prop-c>
 *					<prop-d>value-d</prop-d>
 *				</sub-container>
 *				<prop-e>value-e</prop-e>
 *			</container>
 *		</document>
 *
 *		  example initialization would be:
 *
 *		const TDrivingRule p_rule[] = {
 *			{"document", 0, 1},
 *				{"container", 1, 2}, // 2 is container state
 *					{"prop-a", 2, 20}, // 20 is forst property state
 *					{"prop-b", 2, 21},
 *					{"sub-container", 2, 3},
 *						{"prop-c", 3, 22},
 *						{"prop-d", 3, 23},
 *					{"prop-e", 2, 24}
 *		};
 *		int n_rule_num = 8;
 *		int n_container_state = 2;
 *		int n_property_state = 20;
 *		int n_property_num = 5; // prop-a, prop-b, ... prop-e
 *
 *		- todo: create equivalent constructor with xml input
 */
CXML_FlatItemList_Parser::CXML_FlatItemList_Parser(const TDrivingTable &r_t_table)
	:m_t_table(r_t_table), m_b_dealloc_table(false), m_p_prop_value(0), m_p_prop_specified(0)
{
	m_p_prop_value = new(std::nothrow) wstring[r_t_table.n_property_num];
	m_p_prop_specified = new(std::nothrow) bool[r_t_table.n_property_num];
	// alloc property containers
}

/*
 *	CXML_FlatItemList_Parser::~CXML_FlatItemList_Parser()
 *		- destructor
 */
CXML_FlatItemList_Parser::~CXML_FlatItemList_Parser()
{
	if(m_b_dealloc_table && m_t_table.p_rule)
		delete[] m_t_table.p_rule;
	if(m_p_prop_value)
		delete[] m_p_prop_value;
	if(m_p_prop_specified)
		delete[] m_p_prop_specified;
}

/*
 *	bool CXML_FlatItemList_Parser::b_Status() const
 *		- returns parser status (reflects constructor success)
 *		- returns true if parser was initialized successfuly, otherwise returns false
 */
bool CXML_FlatItemList_Parser::b_Status() const
{
	return m_t_table.p_rule && m_p_prop_value && m_p_prop_specified;
}

/*
 *								=== ~CXML_FlatItemList_Parser ===
 */

/*
 *								=== CXMLNode::CFindByName ===
 */

CXMLNode::CFindByName::CFindByName(const char *p_s_name)
	:m_p_s_name(p_s_name)
{}

bool CXMLNode::CFindByName::operator ()(const CXMLNode *p_node) const
{
	return p_node->n_Type() >= CXMLNode::type_XMLDecl &&
		p_node->n_Type() <= CXMLNode::type_Element &&
		CWideUtils::strmatch_w(p_node->s_Name(), m_p_s_name);
}

/*
 *								=== ~CXMLNode::CFindByName ===
 */

/*
 *								=== CXMLNode::CFindByStlName ===
 */

CXMLNode::CFindByStlName::CFindByStlName(const wstring &r_s_name)
	:m_r_s_name(r_s_name)
{}

bool CXMLNode::CFindByStlName::operator ()(const CXMLNode *p_node) const
{
	return p_node->n_Type() >= CXMLNode::type_XMLDecl &&
		p_node->n_Type() <= CXMLNode::type_Element &&
		p_node->s_Name() == m_r_s_name;
}

/*
 *								=== ~CXMLNode::CFindByStlName ===
 */

/*
 *								=== CXMLNode::CFindElemByName ===
 */

CXMLNode::CFindElemByName::CFindElemByName(const char *p_s_name)
	:m_p_s_name(p_s_name)
{}

bool CXMLNode::CFindElemByName::operator ()(const CXMLNode *p_node) const
{
	return p_node->n_Type() == CXMLNode::type_Element &&
		CWideUtils::strmatch_w(p_node->s_Name(), m_p_s_name);
}

/*
 *								=== ~CXMLNode::CFindElemByName ===
 */

/*
 *								=== CXMLNode::CFindElemByStlName ===
 */

CXMLNode::CFindElemByStlName::CFindElemByStlName(const wstring &r_s_name)
	:m_r_s_name(r_s_name)
{}

bool CXMLNode::CFindElemByStlName::operator ()(const CXMLNode *p_node) const
{
	return p_node->n_Type() == CXMLNode::type_Element &&
		p_node->s_Name() == m_r_s_name;
}

/*
 *								=== ~CXMLNode::CFindElemByStlName ===
 */

/*
 *								=== CXMLNode ===
 */

#ifdef __XML_NODE_USE_DUMMY
#ifdef __XML_NODE_STATIC_DUMMY
CXMLNode CXMLNode::m_dummy(type_DummyNode);
#else // __XML_NODE_STATIC_DUMMY
int CXMLNode::m_n_node_num = 0;
CXMLNode *CXMLNode::m_p_dummy = 0;
#endif // __XML_NODE_STATIC_DUMMY
#endif // __XML_NODE_USE_DUMMY

/*
 *	CXMLNode::CXMLNode(int n_type)
 *		- constructor; creates a new node of type n_type
 *		- this is suitable for type_DummyNode and type_Document only
 */
CXMLNode::CXMLNode(int n_type)
	:m_n_type(n_type)
{
	_ASSERTE(n_type == type_DummyNode || n_type == type_Document);
#if defined(__XML_NODE_USE_DUMMY) && defined(__XML_NODE_STATIC_DUMMY)
	_ASSERTE(n_type != type_DummyNode || this == &m_dummy);
	// user should not construct dummy nodes
#endif // __XML_NODE_USE_DUMMY

#if defined(__XML_NODE_USE_DUMMY) && !defined(__XML_NODE_STATIC_DUMMY)
	++ m_n_node_num;
#endif // __XML_NODE_USE_DUMMY && !__XML_NODE_STATIC_DUMMY
}

/*
 *	CXMLNode::CXMLNode(int n_type, wstring &r_s_value)
 *		- constructor; creates a new node of type n_type and
 *		  value set to r_s_value
 *		- note r_s_value is swapped with this CXMLNode::m_s_value
 *		  and therefore is going to contain empty string after return
 *		- this is suitable for type_Text and type_CData only
 */
CXMLNode::CXMLNode(int n_type, wstring &r_s_value)
	:m_n_type(n_type)
{
	_ASSERTE(n_type == type_Text || n_type == type_CData);
	m_s_value.swap(r_s_value);

#if defined(__XML_NODE_USE_DUMMY) && !defined(__XML_NODE_STATIC_DUMMY)
	++ m_n_node_num;
#endif // __XML_NODE_USE_DUMMY && !__XML_NODE_STATIC_DUMMY
}

/*
 *	CXMLNode::CXMLNode(int n_type, wstring &r_s_name, std::vector<TAttrib> &r_attrib_list)
 *		- costructor; creates a new node of type n_type,
 *		  value set to r_s_value and attributes set to r_attrib_list
 *		- note both r_s_value and r_attrib_list is swapped with this
 *		  CXMLNode::m_s_value and CXMLNode::m_attrib_list, respectively,
 *		  and therefore are both going be empty after return
 *		- this is suitable for type_XMLDecl, type_DocTypeDecl,
 *		  type_ProcInstr and type_Element only
 */
CXMLNode::CXMLNode(int n_type, wstring &r_s_name, std::vector<TAttrib> &r_attrib_list)
	:m_n_type(n_type)
{
	_ASSERTE(n_type >= CXMLNode::type_XMLDecl &&
		n_type <= CXMLNode::type_Element);
	m_s_name.swap(r_s_name);
	m_attrib_list.swap(r_attrib_list);

#if defined(__XML_NODE_USE_DUMMY) && !defined(__XML_NODE_STATIC_DUMMY)
	++ m_n_node_num;
#endif // __XML_NODE_USE_DUMMY && !__XML_NODE_STATIC_DUMMY
}

/*
 *	CXMLNode::~CXMLNode()
 *		- destructor; deletes all subnodes
 */
CXMLNode::~CXMLNode()
{
	std::for_each(m_subnode_list.begin(),
		m_subnode_list.end(), DeleteNode);
	// delete subnodes first !!!

#if defined(__XML_NODE_USE_DUMMY) && !defined(__XML_NODE_STATIC_DUMMY)
	if((-- m_n_node_num) == 1) { // dummy node adds 1 reference
		if(m_p_dummy) {
			delete m_p_dummy;
			m_p_dummy = 0; // !!
		}
	}
#endif // __XML_NODE_USE_DUMMY && !__XML_NODE_STATIC_DUMMY
}

/*
 *	static CXMLNode *CXMLNode::p_Parse(CXML_EventParser &r_parser)
 *		- parses xml file
 *		- note the argument is parser is because of error-checking
 *		  (if error occurs while parsing, caller can refer to
 *		  r_parser error reporting routines)
 *		- returns document node, containing the parsed file on success
 *		  or 0 on failure (this is 0, even when dummy nodes are enabled)
 */
CXMLNode *CXMLNode::p_Parse(CXML_EventParser &r_parser)
{
	CXMLNode *p_doc;
	if(!(p_doc = new(std::nothrow) CXMLNode(type_Document)))
		return 0;
	// create document node

    if(!ParseLoop(p_doc, r_parser)) {
        delete p_doc;
		return 0;
	}
	// parse loop

	return p_doc;
}

/*
 *	const CXMLNode::wstring &CXMLNode::s_InnerText() const
 *		- returns node inner text
 *		- note in case there are subnode elements,
 *		  returned text is just between begin tag of this
 *		  node and begin tag of first child. for example:
 *
 *			<node>text and some more text</node>
 *
 *		  p_node->s_InnerText() returns "text and some more text"
 *
 *			<node>
 *				text
 *				<sub>and some</sub>
 *				more text
 *			</node>
 *
 *		  p_node->s_InnerText() returns only "text"
 */
const CXMLNode::wstring &CXMLNode::s_InnerText() const
{
#ifdef __XML_NODE_USE_DUMMY
	_ASSERTE(p_DummyNode()->m_s_value.empty()); // should be empty
#else // __XML_NODE_USE_DUMMY
	static wstring s_empty("");
#endif // __XML_NODE_USE_DUMMY

	if(m_subnode_list.empty())
#ifdef __XML_NODE_USE_DUMMY
		return p_DummyNode()->m_s_value;
#else // __XML_NODE_USE_DUMMY
		return s_empty;
#endif // __XML_NODE_USE_DUMMY

	const CXMLNode *p_first_child = m_subnode_list.front();
	if(p_first_child->m_n_type == type_Text ||
	   p_first_child->m_n_type == type_CData)
		return p_first_child->m_s_value;

#ifdef __XML_NODE_USE_DUMMY
	return p_DummyNode()->m_s_value;
#else // __XML_NODE_USE_DUMMY
	return s_empty;
#endif // __XML_NODE_USE_DUMMY
}

/*
 *	const CXMLNode::wstring *CXMLNode::p_Attrib(const char *p_s_name) const
 *		- returns value of attribute with name p_s_name,
 *		  or 0 in case there's no attribute with such name
 */
const CXMLNode::wstring *CXMLNode::p_Attrib(const char *p_s_name) const
{
	std::vector<TAttrib>::const_iterator p_attrib_it =
		std::find(m_attrib_list.begin(), m_attrib_list.end(), p_s_name);
	if(p_attrib_it != m_attrib_list.end())
		return &(*p_attrib_it).s_value;
	return 0;
}

/*
 *	const CXMLNode::wstring *CXMLNode::p_Attrib(const wstring &r_s_name) const
 *		- returns value of attribute with name r_s_name,
 *		  or 0 in case there's no attribute with such name
 */
const CXMLNode::wstring *CXMLNode::p_Attrib(const wstring &r_s_name) const
{
	std::vector<TAttrib>::const_iterator p_attrib_it =
		std::find(m_attrib_list.begin(), m_attrib_list.end(), r_s_name);
	if(p_attrib_it != m_attrib_list.end())
		return &(*p_attrib_it).s_value;
	return 0;
}

/*
 *	const CXMLNode *CXMLNode::p_ChildAt(int n_index) const
 *		- returns child node with zero-based index n_index
 *		- if n_index is out of array bounds, returns invalid node
 *		  (invalid node is 0 or dummy node, depending on build config)
 */
const CXMLNode *CXMLNode::p_ChildAt(size_t n_index) const
{
	if(/*n_index >= 0 &&*/ n_index < m_subnode_list.size())
		return m_subnode_list[n_index];
	return p_InvalidNode();
}

/*
 *	const CXMLNode *CXMLNode::p_FirstChild() const
 *		- returns first child node
 *		- in case node list is empty, returns invalid node
 *		  (invalid node is 0 or dummy node, depending on build config)
 */
const CXMLNode *CXMLNode::p_FirstChild() const
{
	if(!m_subnode_list.empty())
		return m_subnode_list.front();
	return p_InvalidNode();
}

/*
 *	const CXMLNode *CXMLNode::p_LastChild() const
 *		- returns last child node
 *		- in case node list is empty, returns invalid node
 *		  (invalid node is 0 or dummy node, depending on build config)
 */
const CXMLNode *CXMLNode::p_LastChild() const
{
	if(!m_subnode_list.empty())
		return m_subnode_list.back();
	return p_InvalidNode();
}

inline void CXMLNode::DeleteNode(CXMLNode *p_node)
{
	delete p_node;
}

#ifdef __XML_NODE_USE_DUMMY
const CXMLNode *CXMLNode::p_DummyNode()
{
#ifdef __XML_NODE_STATIC_DUMMY
	return &m_dummy;
#else // __XML_NODE_STATIC_DUMMY
	if(m_n_node_num > 0) {
		if(!m_p_dummy)
			m_p_dummy = new(std::nothrow) CXMLNode(type_DummyNode);
		return m_p_dummy;
	}
	return 0;
#endif // __XML_NODE_STATIC_DUMMY
}
#endif // __XML_NODE_USE_DUMMY

bool CXMLNode::ParseLoop(CXMLNode *p_doc, CXML_EventParser &r_parser)
{
#if defined(__XML_NODE_USE_DUMMY) && !defined(__XML_NODE_STATIC_DUMMY)
	if(!p_DummyNode())
		return false;
#endif // __XML_NODE_USE_DUMMY && !__XML_NODE_STATIC_DUMMY
	// force dummy node allocation so users don't have to check later

	std::vector<CXMLNode*> node_stack;
	if(!stl_ut::Reserve_N(node_stack, 1))
		return false;
	// node stack

	node_stack.push_back(p_doc);
	// where subnodes go

	CXMLNode *p_parent = p_doc;
	// get parent node

	while(r_parser.Read()) {
		_ASSERTE(!node_stack.empty());

		if(!stl_ut::Reserve_1More(p_parent->m_subnode_list)) {
			delete p_doc;
			return 0;
		}
		// make sure there's space in subnode list

		int n_type;
        switch(n_type = r_parser.n_NodeType()) {
        case CXML_EventParser::node_Text:
        case CXML_EventParser::node_CData:
			{
				CXMLNode *p_new_node;
				if(!(p_new_node = new(std::nothrow) CXMLNode(n_type, r_parser.s_Value())))
					return false;
				// create a new node

				p_parent->m_subnode_list.push_back(p_new_node);
				// add to parent list
			}
			break;
        case CXML_EventParser::node_ProcInstr:
        case CXML_EventParser::node_DocTypeDecl:
        case CXML_EventParser::node_XmlDecl:
        case CXML_EventParser::node_Element:
        case CXML_EventParser::node_SingleElement:
            {
				if(n_type == CXML_EventParser::node_SingleElement)
					n_type = CXML_EventParser::node_Element;
				CXMLNode *p_new_node;
				if(!(p_new_node = new(std::nothrow) CXMLNode(n_type,
				   r_parser.s_Name(), r_parser.r_Attrib_List())))
					return false;
				// create a new node

				p_new_node->m_s_value.swap(r_parser.s_Value());
				// PI parameters are not parsed and are returned trough value
				// for other nodes - value is cleared on the beginning of
				// xml event parser Read() call

				p_parent->m_subnode_list.push_back(p_new_node);
				// add to parent list

                if(r_parser.n_NodeType() == CXML_EventParser::node_Element) {
					if(!stl_ut::Reserve_1More(node_stack))
						return false;
                    node_stack.push_back(p_parent = p_new_node);
				}
                // do not change state for single elements <single-elem/>
            }
            break;
        case CXML_EventParser::node_EndElement:
#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			if(p_parent->m_s_name != r_parser.s_Name())
				return false;
#else // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			_ASSERTE(p_parent->m_s_name == r_parser.s_Name());
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
			node_stack.erase(node_stack.end() - 1);
			p_parent = node_stack.back();
			// remove node from the top
            break;
        case CXML_EventParser::node_Whitespace:
        case CXML_EventParser::node_Comment:
            break; // just silently ignore those
        default:
			return false; // unknown node type
        }
    }

	return node_stack.size() == 1 && node_stack[0] == p_doc &&
		r_parser.n_State() == CXML_EventParser::state_NoError;
}

/*
 *								=== ~CXMLNode ===
 */
