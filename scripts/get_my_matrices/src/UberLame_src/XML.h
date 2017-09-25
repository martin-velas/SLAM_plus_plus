/*
								+----------------------------------+
								|                                  |
								|    ***  Tiny XML parser   ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|              XML.h               |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __XML_PARSER_INCLUDED
#define __XML_PARSER_INCLUDED

/**
 *	@file XML.h
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
 *	t_odo - write DOM parser
 *	@todo - write document writer
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
 *	latest benchmarks show speeds 13MB/s event-based, 11MB/s CXMLNode and 7MB/s TinyXml
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
 *	@date 2011-07-21
 *
 *	Clarified the behavior of __XML_NODE_STATIC_DUMMY in relation with STL's __USE_MALLOC
 *	and CRT memory leak detection.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2012-06-21
 *
 *	Added the CXML_Lexer::ReplaceCharRefs() function to be able to replace character
 *	references in text nodes (note it does not support other than predefined references
 *	at the moment).
 *
 *	The latest benchmarks show speeds 17MB/s event-based, 10MB/s CXMLNode and 7MB/s TinyXml
 *
 *	@todo get rid of Unicode.h and replace it by a simpler reader, based on UniConv.h
 *	@todo write documentation comments
 *	@todo add an option to use segregated storage for the XML nodes, should get some
 *		additional speed to CXMLNode parser (and forward-allocated is ok for read-only DOM).
 *
 *	Removed some of CWideUtils members, use __DEPRECATED_WIDE_UTILS to compile them.
 *
 *	Added the __XML_CASE_SENSITIVE_ATTRIBS macro.
 *
 */

/**
 *	@def __XML_CASE_SENSITIVE_ATTRIBS
 *	@brief if defined, attribute name comparison will be case-sensitive
 *		(otherwise __DEPRECATED_WIDE_UTILS needs to be defined)
 */
#define __XML_CASE_SENSITIVE_ATTRIBS

/**
 *	@def __DEPRECATED_WIDE_UTILS
 *	@brief compiles some of CWideUtils members, which have been unused and deprecated
 */
//#define __DEPRECATED_WIDE_UTILS

/**
 *	@def __XML_LEXER_NORMALIZE_NEWLINES
 *	@brief enabling this macro makes XML lexer normalize newlines from '\r', '\n' or '\r\n' to '\n'
 *	@note If disabled, lexer (and any dependent parsers) is no longer XML compliant,
 *		but is faster and smaller.
 */
//#define __XML_LEXER_NORMALIZE_NEWLINES

/**
 *	@def __XML_LEXER_TOKEN_DUMP
 *	@brief if defined, CXML_Lexer::TToken has got member function Dump()
 *		which prints token contents to stdout (since this is not normally
 *		needed, it isn't compiled by default)
 */
//#define __XML_LEXER_TOKEN_DUMP

/**
 *	@def __XML_LEXER_XML10_COMPLIANT_CHARSET
 *	@brief if defined, lexer only accepts characters specified as valid by XML 1.0
 *		(on the other side, this bloats lexer tables approximately ten times and
 *		slows down (about 33%) even if it's not really needed in most cases;
 *		if not defined, any character in range 0 - 0x10ffff is considered valid)
 */
//#define __XML_LEXER_XML10_COMPLIANT_CHARSET

/**
 *	@def __XML_LEXER_COMPACT_DRIVING_TABLE
 *	@brief chooses between compact driving table and 32-bit aligned driving table,
 *		effectively optimizing for speed / size (may depend on target architecture)
 */
//#define __XML_LEXER_COMPACT_DRIVING_TABLE

/**
 *	@def __XML_LEXER_FORMAT_UTF_32
 *	@brief sets xml lexer output string format to raw unicode, stored as 32 bit integer
 *		(sufficient for the whole unicode range)
 */
//#define __XML_LEXER_FORMAT_UTF_32

/**
 *	@def __XML_LEXER_FORMAT_RAW_16
 *	@brief sets xml lexer output string format to raw unicode, stored as 16 bit integer
 *		(sufficient for central european)
 */
//#define __XML_LEXER_FORMAT_RAW_16

/**
 *	@def __XML_LEXER_FORMAT_RAW_8
 *	@brief sets xml lexer output string format to raw unicode, stored as 8 bit integer
 *		(sufficient for us-english)
 */
//#define __XML_LEXER_FORMAT_RAW_8

/**
 *	@def __XML_LEXER_FORMAT_UTF_16
 *	@brief sets xml lexer output string format to UTF-16, little endian
 *	@note UTF-16 string length can be larger than actual number of characters.
 */
//#define __XML_LEXER_FORMAT_UTF_16

/**
 *	@def __XML_LEXER_FORMAT_UTF_8
 *	@brief sets xml lexer output string format to UTF-8
 *	@note UTF-8 string length can be larger than actual number of characters.
 */
#define __XML_LEXER_FORMAT_UTF_8

/**
 *	@def __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
 *	@brief if defined, xml parser does a few checks about document well-formedness, one of them
 *		being attributes must have unique names inside a single element which slows down
 *	@note If disabled, event-based parser (and any dependent) is no longer XML compliant,
 *		but is faster and smaller.
 */
//#define __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

/**
 *	@def __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS
 *	@brief if defined, encodings in form win-1234 or windows-1234 are recognized as single
 *		byte encodings (otherwise the parser won't read file with such an encoding)
 */
#define __XML_PARSER_SUPPORT_WINDOWS_ENCODINGS

/**
 *	@def __XML_PARSER_IGNORE_UNKNOWN_ENCODINGS
 *	@brief if defined and an unknown encoding is encountered, it is regarded as 8-bit ascii
 */
#define __XML_PARSER_IGNORE_UNKNOWN_ENCODINGS

#include <vector>
#include <string>
#include <algorithm>
#include "Scanner.h"
#include "Unicode.h"
#include "Unused.h"

/**
 *	@brief simple table-driven lexical analyzer for XML
 */
class CXML_Lexer {
public:
	/**
	 *	@brief token type names (negative ones mark error)
	 */
	enum {
		error_CharRef = -3, /**< @brief invalid character references (attribute value) */
		error_Memory = -2, /**< @brief not enough memory for string */
		error_Code = -1, /**< @brief char value too big for string representation */
		token_White = 0, /**< @brief whitespace */
		token_Comment, /**< @brief comment */
		token_CData, /**< @brief CDATA section */
		token_Elem, /**< @brief < */
		token_EndElem, /**< @brief </ */
		token_PIElem, /**< @brief <? */
		token_DTDElem, /**< @brief <! */
		token_End, /**< @brief > */
		token_SingleEnd, /**< @brief /> */
		token_PIEnd, /**< @brief ?> */
		token_Assign, /**< @brief = */
		token_Name, /**< @brief name of the tag */
		token_AttValue, /**< @brief strings with &amp; &\#[0-9]+; &\#x[0-9a-fA-F]+; references replaced */
		token_EOF /**< @brief end of file (special token) */
	};

#if defined(__XML_LEXER_FORMAT_UTF_32)
	typedef int TCharType; /**< @brief wide character data type */
#elif defined(__XML_LEXER_FORMAT_RAW_16) || defined(__XML_LEXER_FORMAT_UTF_16)
	typedef unsigned short TCharType; /**< @brief wide character data type */
#elif defined(__XML_LEXER_FORMAT_RAW_8) || defined(__XML_LEXER_FORMAT_UTF_8)
	typedef char TCharType; /**< @brief character data type */
#else
#error "error: none of __XML_LEXER_FORMAT_* was defined. see XML.h"
#endif

	typedef std::basic_string<TCharType> wstring; /**< @brief wide-character string type */

	/**
	 *	@brief simple token structure
	 */
	struct TToken {
		int n_line; /**< @brief line where the token occurs */
		int n_col; /**< @brief column where the token occurs */
		int n_type; /**< @brief token type */
		wstring s_value; /**< @brief string value of the token (may be empty) */

		/**
		 *	@brief creates a new token
		 *
		 *	@param[in] _n_line is line where the token occurs
		 *	@param[in] _n_col is column where the token occurs
		 *	@param[in] _n_type is token type 
		 */
		inline TToken(int _n_line, int _n_col, int _n_type);

		/**
		 *	@brief creates a new token
		 *
		 *	@param[in] _n_line is line where the token occurs
		 *	@param[in] _n_col is column where the token occurs
		 *	@param[in] _n_type is token type 
		 *	@param[in] p_s_buffer is unicode (UTF-32) string, containing token contents
		 *
		 *	@note If there is not enough memory to create copy of the string,
		 *		token type is set to error_Memory. If code of characters in p_s_buffer
		 *		is greater than TCharType can contain, token type is set to error_Code.
		 */
		TToken(int _n_line, int _n_col, int _n_type, const int *p_s_buffer);

		/**
		 *	@brief creates a new token
		 *
		 *	@param[in] _n_line is line where the token occurs
		 *	@param[in] _n_col is column where the token occurs
		 *	@param[in] _n_type is token type
		 *	@param[in] p_s_buffer is unicode (UTF-32) string, containing token contents
		 *	@param[in] n_length is length of string in p_s_buffer
		 *
		 *	@note If there is not enough memory to create copy of the string,
		 *		token type is set to error_Memory. If code of characters in p_s_buffer
		 *		is greater than TCharType can contain, token type is set to error_Code.
		 */
		TToken(int _n_line, int _n_col, int _n_type, const int *p_s_buffer, size_t n_length);

#ifdef __XML_LEXER_TOKEN_DUMP
		/**
		 *	@brief prints token contents to stdout
		 *	@note This is compiled only if __XML_LEXER_TOKEN_DUMP is defined.
		 */
		void Dump() const;
#endif // __XML_LEXER_TOKEN_DUMP
	};

protected:
	typedef bool (*TokenEmitFunc)(const int *p_s_buffer,
		int n_regexp_id, int n_line, int n_column, TToken *p_out_token);

	class CTokenEmitAdapter {
	protected:
		TToken *m_p_out_token;
		TokenEmitFunc *m_p_emit_func;

	public:
		inline CTokenEmitAdapter(TToken &r_t_out_token, TokenEmitFunc *p_emit_func);
		inline bool operator ()(const int *p_s_buffer,
			int n_regexp_id, int n_line, int n_column) const;
	};

	class CFileReader {
	protected:
		CUnicodeFile m_file;
		int m_n_cached_char;
		int m_n_io_error;

	public:
		CFileReader(const char *p_s_filename);
		~CFileReader();
		bool SetEncoding(int n_encoding);
		bool b_Status() const;
		int n_IO_Error() const;
		inline bool ReadChar(int &r_n_char);
		inline bool UnreadChar(int n_char);
	};

protected:
#ifdef __XML_LEXER_COMPACT_DRIVING_TABLE
#pragma pack(1)
	struct TTransition {
		uint32_t n_char_max;
		uint16_t n_char_min;
		int16_t n_state;
	};

	struct TState {
		const TTransition *p_transition;
		int8_t n_regexp_id;
		int16_t n_transition_num;
	};
#pragma pack()
#else // __XML_LEXER_COMPACT_DRIVING_TABLE
	struct TTransition {
		uint32_t n_char_max;
		uint32_t n_char_min;
		int32_t n_state;
	};

	struct TState {
		const TTransition *p_transition;
		int32_t n_regexp_id;
		int32_t n_transition_num;
	};
#endif // __XML_LEXER_COMPACT_DRIVING_TABLE

	static const TTransition m_p_transition[];
	static const TState m_p_state[];
	CScanner<int, TState, TTransition, CTokenEmitAdapter> m_scanner;
	static TokenEmitFunc m_p_emit_func_list[];
	CFileReader *m_p_file;
	TToken m_t_token;

public:
	/*
	 *	CXML_Lexer::CXML_Lexer(const char *p_s_filename)
	 *		- opens a new file p_s_filename
	 *		- call b_Status() to see wheter constructor succeeded
	 */
	CXML_Lexer(const char *p_s_filename);

	/*
	 *	CXML_Lexer::~CXML_Lexer()
	 *		- destructor
	 */
	~CXML_Lexer();

	/*
	 *	bool CXML_Lexer::b_Status() const
	 *		- returns status of lexer
	 *		- returns true if constructor successfuly opened the file, otherwise returns false
	 *		- doesn't indicate parsing errors
	 */
	bool b_Status() const;

	/*
	 *	int CXML_Lexer::n_IO_Error() const
	 *		- returns CUnicodeFile error codes
	 *		- always returns CUnicodeFile::error_InvalidOperation if no file was opened
	 */
	int n_IO_Error() const;

	/*
	 *	bool CXML_Lexer::SetEncoding(int n_encoding = CUnicodeFile::code_UTF_8)
	 *		- force some kind of encoding (after reading <?xml version="1.0" encoding="..."?>)
	 *		- n_encoding is one of CUnicodeFile::code_ASCII, code_UTF_8 or code_UTF_16_LE
	 */
	bool SetEncoding(int n_encoding = CUnicodeFile::code_UTF_8);

	/*
	 *	inline int CXML_Lexer::n_PeekChar()
	 *		- peeks at the next character in the stream
	 *		- returns character code or -1 on error / EOF
	 */
	inline int n_PeekChar();

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
	bool Get_NextToken();

	/*
	 *	bool CXML_Lexer::Get_Text(wstring &r_s_result, bool b_erase_result)
	 *		- reads text data, ending with end of file or the < character
	 *		- if b_erase_result is set to true, r_s_result is erased before reading anything
	 *		- may contain &amp; or &#xABCD; character references
	 *		- returns true on success, false on failure
	 *		- note t_Cur_Token() return value is unchanged by call to this function
	 */
	bool Get_Text(wstring &r_s_result, bool b_erase_result);

	/*
	 *	bool CXML_Lexer::Get_PIData(wstring &r_s_result)
	 *		- reads processing instruction data, ending with the ?> sequence
	 *		- the ?> sequence is not contained in r_s_result,
	 *		  but is parsed (there won't be token_PIEnd)
	 *		- returns true on success, false on failure
	 *		- note t_Cur_Token() return value is unchanged by call to this function
	 */
	bool Get_PIData(wstring &r_s_result);

	/*
	 *	inline int CXML_Lexer::n_Cur_Line() const
	 *		- returns zero-based index of current line in input file
	 */
	inline int n_Cur_Line() const;

	/*
	 *	inline int CXML_Lexer::n_Cur_Column() const
	 *		- returns zero-based index of current column in input file
	 */
	inline int n_Cur_Column() const;

	/*
	 *	inline CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token()
	 *		- returns value of current token
	 */
	inline CXML_Lexer::TToken &t_Cur_Token();

	/*
	 *	inline const CXML_Lexer::TToken &CXML_Lexer::t_Cur_Token() const
	 *		- returns value of current token
	 */
	inline const CXML_Lexer::TToken &t_Cur_Token() const;

	/*
	 *	inline int CXML_Lexer::n_Cur_Token() const
	 *		- returns type of current token
	 */
	inline int n_Cur_Token() const;

	/*
	 *	static bool CXML_Lexer::ReplaceCharRefs(int *p_s_buffer)
	 *		- replaces character references such as &amp; &#0123; or &#xABCD;
	 */
	static bool ReplaceCharRefs(int *p_s_buffer);

	/*
	 *	static bool CXML_Lexer::ReplaceCharRefs(wstring &r_s_string)
	 *		- replaces character references such as &amp; &#0123; or &#xABCD;
	 */
	static bool ReplaceCharRefs(wstring &r_s_string);

#ifdef __XML_LEXER_NORMALIZE_NEWLINES
	/*
	 *	static void CXML_Lexer::NormalizeNewlines(int *p_s_buffer)
	 *		- normalizes newlines in zero-terminated unicode string p_s_buffer
	 *		  (finds all occurences of \r\n or \r and replaces them by \n)
	 *		- note this doesn't really find substrings, it's somewhat optimized
	 */
	static void NormalizeNewlines(int *p_s_buffer);
#endif // __XML_LEXER_NORMALIZE_NEWLINES

	/*
	 *	static bool CXML_Lexer::AppendChar(wstring &r_s_value, int n_char)
	 *		- append char n_char (raw 32-bit unicode) to string r_s_value,
	 *		  while applying charset conversion, given by __XML_LEXER_FORMAT_*
	 *		- returns 0 on success, -1 on code conversion error or
	 *		  -2 on insufficient memory error
	 */
	static int AppendChar(wstring &r_s_value, int n_char);

protected:
	static bool EmitWhitespace(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitComment(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitCData(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitElem(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitOperator(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitName(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
	static bool EmitAttValue(const int *p_s_buffer, int n_regexp_id, int n_line, int n_column, TToken *p_out_token);
};

/*
 *	class CWideUtils
 *		- wide character utility functions
 */
class CWideUtils {
public:
	static size_t strlen_w(const int *p_s_buffer);
	// utility function returning length of unicode string (note this can't be replaced by wcslen())

#ifdef __DEPRECATED_WIDE_UTILS
	static char *strconv_w(const CXML_Lexer::wstring &r_s_string);
	static char *strconv_w(const int *p_s_buffer);
	static bool strconv_w(std::string &r_s_dest, const CXML_Lexer::wstring &r_s_string);
	static bool strconv_w(std::string &r_s_dest, const int *p_s_buffer);
	// utility functions, converting wide string to normal string
	// (characters must be ascii, otherwise returns 0 - or fails respectively)

	static char *strconv_w(const CXML_Lexer::wstring &r_s_string, char n_replacement);
	static char *strconv_w(const int *p_s_buffer, char n_replacement);
	static bool strconv_w(std::string &r_s_dest,
		const CXML_Lexer::wstring &r_s_string, char n_replacement);
	static bool strconv_w(std::string &r_s_dest,
		const int *p_s_buffer, char n_replacement);
	// utility functions, converting wide string to normal string
	// (character codes above 255 are replaced by n_replacement)

	static long atol_w(const int *p_s_buffer);
	static int atoi_w(const int *p_s_buffer);
	static long atol_w(const CXML_Lexer::wstring &r_s_string);
	static int atoi_w(const CXML_Lexer::wstring &r_s_string);
	// wide string conversion to number; overflow not detected
	// in case no numerich character occured, returns 0 (but skips whitespace on beginning)

	static bool strinmatch_w(const int *p_s_buffer, const char *p_s_match, size_t n_length);
	static bool strimatch_w(const int *p_s_buffer, const char *p_s_match);
	static bool strimatch_w(const CXML_Lexer::wstring &r_s_string, const char *p_s_name);
#endif // __DEPRECATED_WIDE_UTILS

	static bool strmatch_w(const CXML_Lexer::wstring &r_s_string, const char *p_s_name);
	// utitity functions for comparing unicode and ascii strings
};


/*
 *	class CXML_EventParser
 *		- simple not-really-event-based xml parser
 *		- Microsoft's c\# System.Xml.XmlTextReader was model for this
 */
class CXML_EventParser {
public:
	/*
	 *	enum CXML_EventParser::<__unnamed_1>
	 *		- node type names
	 */
	enum {
		node_None = -1,			// parser returns this if no node has been read. call Read()
		node_XmlDecl,			// <?xml version="1.0" encoding="utf-8" ?> declaration
		node_DocTypeDecl,		// <!DOCUMENT > without parameters (they're ignored)
		node_ProcInstr,			// <?PI ?> with unparsed optional parameters in value
		node_Element,			// <element > with attributes
		node_SingleElement,		// <element /> with attributes
		node_EndElement,		// </element>
		node_CData,				// CData contents (in value)
		node_Text,				// text data between markup (in value)
		node_Comment,			// comment (in value)
		node_Whitespace			// whitespace (in value)
	};

	/*
	 *	enum CXML_EventParser::<__unnamed_2>
	 *		- parser state names
	 */
	enum {
		state_NoError,			// no error
		state_NoMemory,			// not enough memory
		state_BadEncoding,		// unsupported encoding specified in xml declaration
		state_CodeError,		// error while converting unicode to CXML_Lexer::TCharType
		state_LexerInitError,	// failed to open input file
		state_LexerError,		// lexer error (failed to initialize i/o, ...)
		state_LexicalError,		// lexical error (error while parsing tokens)
		state_UnexpectedEOF,	// unexpected end of file
		state_UnexpectedToken,	// unexpected token
		state_MalformedDoc,		// well-formendes constraint broken
		state_UnsupportedNode	// encountered node which is not supported by parser
	};

	/*
	 *	class CXML_EventParser::wstring
	 *		- wide-character string type (specialization of std::basic_string)
	 */
	typedef CXML_Lexer::wstring wstring;

	/*
	 *	struct CXML_EventParser::TAttrib
	 *		- xml element attribute, consists of name and the assigned value
	 */
	struct TAttrib {
		wstring s_name;
		wstring s_value;

		inline bool operator ==(const wstring &r_s_name) const
		{
			return s_name == r_s_name;
		}

		inline bool operator ==(const char *p_s_name) const
		{
			return CWideUtils::strmatch_w(s_name, p_s_name);
		}
	};

protected:
	CXML_Lexer m_lexer;

	int m_n_node_type;
	int m_n_state;

	wstring m_s_name, m_s_value;
	std::vector<TAttrib> m_attrib_list;

#ifdef __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS
	std::vector<wstring> m_node_stack;
#endif // __XML_PARSER_CHECK_DOCUMENT_WELL_FORMEDNESS

public:
	/*
	 *	CXML_EventParser::CXML_EventParser(const char *p_s_filename)
	 *		- constructor; creates parser and opens file p_s_filename
	 *		- it's necessary to call Read() to read the first (and any following) node
	 */
	CXML_EventParser(const char *p_s_filename);

	/*
	 *	int CXML_EventParser::n_State() const
	 *		- returns parser state (but does not clear it)
	 *		- if no error occured, returns state_NoError
	 */
	int n_State() const;

	/*
	 *	const char *CXML_EventParser::p_s_State() const
	 *		- returns string description of parser state
	 */
	const char *p_s_State() const;

	/*
	 *	int CXML_EventParser::n_Cur_Line() const
	 *		- returns zero-based index of current line in input file
	 */
	int n_Cur_Line() const;

	/*
	 *	int CXML_EventParser::n_Cur_Column() const
	 *		- returns zero-based index of current column in input file
	 */
	int n_Cur_Column() const;

	/*
	 *	bool CXML_EventParser::Read()
	 *		- reads an xml node
	 *		- use n_NodeType() to determine what kind of node was just read
	 *		- returns true on success and false on EOF / failure
	 *		  (use n_State() to decide between EOF and failure)
	 *		- note this always fails if parser is in error state (so it's not
	 *		  necessary to call n_State() unless Read() fails)
	 */
	bool Read();

	/*
	 *	int CXML_EventParser::n_NodeType() const
	 *		- returns type of last parsed node
	 *		- returns node_None if Read() has not been called yet
	 *		- note the return value is invalid if parser is in error state
	 */
	int n_NodeType() const;

	/*
	 *	const CXML_EventParser::wstring &CXML_EventParser::s_Name() const
	 *		- returns name of last parsed node (only node_XmlDecl, node_DocTypeDecl,
	 *		  node_ProcInstr, node_Element and node_SingleElement have names)
	 */
	const wstring &s_Name() const;

	/*
	 *	CXML_EventParser::wstring &CXML_EventParser::s_Name()
	 *		- returns name of last parsed node (only node_XmlDecl, node_DocTypeDecl,
	 *		  node_ProcInstr, node_Element and node_SingleElement have names)
	 */
	wstring &s_Name();

	/*
	 *	const CXML_EventParser::wstring &CXML_EventParser::s_Value() const
	 *		- returns value of last parsed node (only node_ProcInstr, node_CData,
	 *		  node_Text, node_Comment and node_Whitespace have values)
	 *		- note node_Text may contain character references such as &amp; or &#xABCD;
	 *		  which haven't been replaced (there might be external character references)
	 *		  use CXML_Lexer::ReplaceCharRefs() to replace them.
	 */
	const wstring &s_Value() const;

	/*
	 *	CXML_EventParser::wstring &CXML_EventParser::s_Value()
	 *		- returns value of last parsed node (only node_ProcInstr, node_CData,
	 *		  node_Text, node_Comment and node_Whitespace have values)
	 *		- note node_Text may contain character references such as &amp; or &#xABCD;
	 *		  which haven't been replaced (there might be external character references)
	 *		  use CXML_Lexer::ReplaceCharRefs() to replace them.
	 */
	wstring &s_Value();

	/*
	 *	const std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List() const
	 *		- returns const reference to attribute list of last parsed node
	 */
	const std::vector<TAttrib> &r_Attrib_List() const;

	/*
	 *	std::vector<CXML_EventParser::TAttrib> &CXML_EventParser::r_Attrib_List()
	 *		- returns reference to attribute list of last parsed node
	 */
	std::vector<TAttrib> &r_Attrib_List();

	/*
	 *	int CXML_EventParser::n_Attribute_Num() const
	 *		- returns number of node attributes (only node_XmlDecl, node_Element and
	 *		  node_SingleElement may have parameters)
	 *		- note node_ProcInstr may have parameters as well, but those are not parsed and
	 *		  are all returned as string by s_Value()
	 *		- note mode_DocTypeDecl may have parameters, but it's not spported at the moment
	 */
	size_t n_Attribute_Num() const;

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
	const TAttrib &t_Attribute(size_t n_index) const;

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
	TAttrib &t_Attribute(size_t n_index);

	/*
	 *	const CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(
	 *		const char *p_s_name) const
	 *		- returns attribute with name equal to p_s_name or 0 if there is no such attribute
	 */
	const TAttrib *p_AttributeByName(const char *p_s_name) const;

	/*
	 *	CXML_EventParser::TAttrib *CXML_EventParser::p_AttributeByName(const char *p_s_name)
	 *		- returns attribute with name equal to p_s_name or 0 if there is no such attribute
	 */
	TAttrib *p_AttributeByName(const char *p_s_name);

	/*
	 *	static const char *CXML_EventParser::p_s_ErrorString(int n_state)
	 *		- returns string representation of parser state n_state
	 */
	static const char *p_s_ErrorString(int n_state);

protected:
	bool Parse_Element();
	bool Parse_Attribs();
	bool Parse_ProcInstr();
	bool Parse_DTD();
	bool ReserveAttrib();
	int n_GetTokenErrorType();
	int n_GetUnexpectedErrorType();
	bool Detect_Set_Encoding();
	static inline bool b_IsEncodingAttrib(const TAttrib &r_attrib);
};

/*
 *	class CXML_FlatItemList_Parser
 *		- simple and fast parser for xml's containing not-tree-hierarchical lists of items
 *		  such as (structured) database tables (the opposite would be directory structure)
 */
class CXML_FlatItemList_Parser {
public:
	/*
	 *	struct CXML_FlatItemList_Parser::TDrivingRule
	 *		- a single rule in driving table
	 */
    struct TDrivingRule {
        const char *p_s_name; // node name (todo - maybe do it in unicode)
        int n_parent_state, n_state; // in state and out state
    };

	/*
	 *	struct CXML_FlatItemList_Parser::TDrivingTable
	 *		- parser driving table
	 */
	struct TDrivingTable {
		TDrivingRule *p_rule; // array of driving rules
		int n_rule_num; // number of driving rules

		int n_container_state; // state parser comes into when encountering container element
		int n_property_state; // state parser comes into when encountering first property element
		int n_property_num; // number of property elements
	};

	typedef CXML_EventParser::wstring wstring; // wide string

protected:
	TDrivingTable m_t_table;
	bool m_b_dealloc_table;
	wstring *m_p_prop_value;
	bool *m_p_prop_specified;

public:
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
	CXML_FlatItemList_Parser(const char **p_container_path, int n_container_path_length,
		const char **p_property_name_list, int n_property_num);

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
	CXML_FlatItemList_Parser(const TDrivingTable &r_t_table);

	/*
	 *	CXML_FlatItemList_Parser::~CXML_FlatItemList_Parser()
	 *		- destructor
	 */
	~CXML_FlatItemList_Parser();

	/*
	 *	bool CXML_FlatItemList_Parser::b_Status() const
	 *		- returns parser status (reflects constructor success)
	 *		- returns true if parser was initialized successfuly, otherwise returns false
	 */
	bool b_Status() const;

	/*
	 *	template <class CEmitObject>
     *	bool CXML_FlatItemList_Parser::Parse(CXML_EventParser &parser, CEmitObject emit_container)
	 *		- parses the rest of an xml file opened by parser
	 *		- emit_container is function object which gets array of wstring with property values
	 *		  and array of booleans indicating wheter corresponding property was
	 *		  specified (true) or not (false)
	 *		- note this is error-prone because order of property names emit_container assumes must
	 *		  be the same as specified in constructor and there's no mechanism which would enforce it
	 *		- emit_container is called upon each </container> element and must return true
	 *		  if parsing should continue or false if parsing is required to fail immediately
	 *		- returns true on success, false on failure
	 */
    template <class CEmitObject>
    bool Parse(CXML_EventParser &parser, CEmitObject emit_container)
    {
        const TDrivingRule *p_driving_table = m_t_table.p_rule;
        const int n_driving_table_size = m_t_table.n_rule_num;
        // driving table

        const int n_container_state = m_t_table.n_container_state;
        const int n_property_state = m_t_table.n_property_state;
		// container / first property state

        const int n_property_num = m_t_table.n_property_num;
        // number of prop types

        int n_state = 0;
        // parsr state

        while(parser.Read()) {
            switch(parser.n_NodeType()) {
            case CXML_EventParser::node_Element:
            case CXML_EventParser::node_SingleElement:
                {
                    int n_next_state = -1;
                    for(int i = 0; i < n_driving_table_size; ++ i) {
                        if(n_state == p_driving_table[i].n_parent_state &&
                           CWideUtils::strmatch_w(parser.s_Name(), p_driving_table[i].p_s_name)) {
                            n_next_state = p_driving_table[i].n_state;
                            break;
                        }
                    }
                    // find if element fits DTD

                    if(n_next_state == n_container_state) { // container begins
                        memset(m_p_prop_specified, 0, n_property_num * sizeof(bool));
                        // clear property flags
                    } else if(n_next_state >= n_property_state &&
                       n_next_state <= n_property_state + n_property_num) {
                        m_p_prop_specified[n_next_state - n_property_state] = false; // property was not yet specified
                        //m_p_prop_value[n_next_state - n_property_state] = ""; // initialize it's value
                    } else if(n_next_state < 0) {
                        return false;
                        // element doesn't fit DTD
                    }
                    // react to state

                    if(parser.n_NodeType() == CXML_EventParser::node_Element)
                        n_state = n_next_state;
                    // do not change state for single elements <single-elem/>
                }
                break;
            case CXML_EventParser::node_EndElement:
                {
                    if(n_state == n_container_state) {
                        if(!emit_container(m_p_prop_specified, m_p_prop_value))
                            return false;
                        // container parsing was finished, pass it to user function
                    }
                    // flush container items

                    int n_next_state = -1;
                    for(int i = 0; i < n_driving_table_size; ++ i) {
                        if(n_state == p_driving_table[i].n_state &&
                           CWideUtils::strmatch_w(parser.s_Name(), p_driving_table[i].p_s_name)) {
                            n_next_state = p_driving_table[i].n_parent_state;
                            break;
                        }
                    }
                    // find if element fits DTD

                    if(n_next_state < 0) {
                        return false;
                        // element doesn't fit DTD
                    }
                    // react to state

                    n_state = n_next_state;
                }
                break;
            case CXML_EventParser::node_Text:
            case CXML_EventParser::node_CData:
                if(n_state >= n_property_state &&
                   n_state <= n_property_state + n_property_num) {
                    if(m_p_prop_specified[n_state - n_property_state])
                        return false; // multiple types of specification in a single node
                    m_p_prop_specified[n_state - n_property_state] = true;
                    m_p_prop_value[n_state - n_property_state].swap(parser.s_Value());
                } else {
                    return false;
                    // unexpected value
                }
                break;
            case CXML_EventParser::node_Whitespace:
            case CXML_EventParser::node_Comment:
            case CXML_EventParser::node_ProcInstr:
            case CXML_EventParser::node_DocTypeDecl:
            case CXML_EventParser::node_XmlDecl:
                break; // just silently ignore those
            default:
                return false; // unknown node type
            }
        }

        if(n_state != 0 || parser.n_State() != CXML_EventParser::state_NoError)
            return false;

        return true;
    }
};

/**
 *	@def __XML_NODE_USE_DUMMY
 *	@brief if defined, CXMLNode members p_FindChild() and p_FindNextChild()
 *		doesn't return null in case specified child node is not found
 *		(pointer to empty dummy node of type type_DummyNode is returned instead)
 */
#define __XML_NODE_USE_DUMMY

/**
 *	@def __XML_NODE_STATIC_DUMMY
 *	@brief if defined, dummy node is defined as static (no reference counting code required)
 *	@note The dummy node is going to contain two std::string members which remain
 *		allocated, adding to memory leaks (even though __USE_MALLOC is used).
 */
//#define __XML_NODE_STATIC_DUMMY

/**
 *	@brief XML parse tree node
 */
class CXMLNode {
public:
	/**
	 *	@brief XML node types
	 */
	enum {
		type_DummyNode = -1, /**< @brief dummy node (has no value nor children) */
		type_XMLDecl = CXML_EventParser::node_XmlDecl, /**< @brief XML declaration node */
		type_DocTypeDecl = CXML_EventParser::node_DocTypeDecl, /**< @brief DTD node */
		type_ProcInstr = CXML_EventParser::node_ProcInstr, /**< @brief processing instruction node */
		type_Element = CXML_EventParser::node_Element, /**< @brief element node (use s_Name() to get tag) */
		type_CData = CXML_EventParser::node_CData, /**< @brief CDATA node (use s_Value() to get the data) */
		type_Text = CXML_EventParser::node_Text, /**< @brief text node (use s_Value() to get text) */
		type_Document /**< @brief document (root node) */
	};

	typedef CXML_EventParser::wstring wstring; /**< @brief wide string type */
	typedef CXML_EventParser::TAttrib TAttrib; /**< @brief element attribute */
	typedef std::vector<CXMLNode*>::const_iterator _node_it; /**< @brief node iterator */

protected:
	int m_n_type; /**< @brief node type (one of type_*) */
	wstring m_s_name; /**< @brief node name (in case of element node) */
	wstring m_s_value; /**< @brief node value (in case of text / CDATA node) */
	std::vector<TAttrib> m_attrib_list; /**< @brief list of element attributes */
	std::vector<CXMLNode*> m_subnode_list; /**< @brief list of children */

#ifdef __XML_NODE_USE_DUMMY
#ifdef __XML_NODE_STATIC_DUMMY
	static CXMLNode m_dummy; /**< @brief static dummy node */
#else // __XML_NODE_STATIC_DUMMY
	static int m_n_node_num; /**< @brief reference counter for the dummy node */
	static CXMLNode *m_p_dummy; /**< @brief reference-counted dynamically allocated dummy node */
#endif // __XML_NODE_STATIC_DUMMY
#endif // __XML_NODE_USE_DUMMY

	class CFindByName {
	protected:
		const char *m_p_s_name;
	public:
		CFindByName(const char *p_s_name);
		bool operator ()(const CXMLNode *p_node) const;
	};
	class CFindByStlName {
	protected:
		const wstring &m_r_s_name;
	public:
		CFindByStlName(const wstring &r_s_name);
		bool operator ()(const CXMLNode *p_node) const;
	};
	class CFindElemByName {
	protected:
		const char *m_p_s_name;
	public:
		CFindElemByName(const char *p_s_name);
		bool operator ()(const CXMLNode *p_node) const;
	};
	class CFindElemByStlName {
	protected:
		const wstring &m_r_s_name;
	public:
		CFindElemByStlName(const wstring &r_s_name);
		bool operator ()(const CXMLNode *p_node) const;
	};

public:
	/*
	 *	CXMLNode::CXMLNode(int n_type)
	 *		- constructor; creates a new node of type n_type
	 *		- this is suitable for type_DummyNode and type_Document only
	 */
	CXMLNode(int n_type);

	/*
	 *	CXMLNode::CXMLNode(int n_type, wstring &r_s_value)
	 *		- constructor; creates a new node of type n_type and
	 *		  value set to r_s_value
	 *		- note r_s_value is swapped with this CXMLNode::m_s_value
	 *		  and therefore is going to contain empty string after return
	 *		- this is suitable for type_Text and type_CData only
	 */
	CXMLNode(int n_type, wstring &r_s_value);

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
	CXMLNode(int n_type, wstring &r_s_name, std::vector<TAttrib> &r_attrib_list);

	/*
	 *	CXMLNode::~CXMLNode()
	 *		- destructor; deletes all subnodes
	 */
	~CXMLNode();

	/*
	 *	static CXMLNode *CXMLNode::p_Parse(CXML_EventParser &r_parser)
	 *		- parses xml file
	 *		- note the argument is parser is because of error-checking
	 *		  (if error occurs while parsing, caller can refer to
	 *		  r_parser error reporting routines)
	 *		- returns document node, containing the parsed file on success
	 *		  or 0 on failure (this is 0, even when dummy nodes are enabled)
	 */
	static CXMLNode *p_Parse(CXML_EventParser &r_parser);

	/*
	 *	inline int CXMLNode::n_Type() const
	 *		- returns node type (one of CXMLNode::<__unnamed_1>)
	 */
	inline int n_Type() const
	{
		return m_n_type;
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_ToElement() const
	 *		- returns pointer to this if this node is element.
	 *		  otherwise returns invalid node (invalid node is 0 or dummy node,
	 *		  depending on build config)
	 */
	inline const CXMLNode *p_ToElement() const
	{
		return (m_n_type == type_Element)? this : p_InvalidNode();
	}

	inline bool b_IsElem() const
	{
		return m_n_type == type_Element;
	}

	inline bool b_IsElement() const
	{
		return m_n_type == type_Element;
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_ToText() const
	 *		- returns pointer to this if this node is text.
	 *		  otherwise returns invalid node (invalid node is 0 or dummy node,
	 *		  depending on build config)
	 */
	inline const CXMLNode *p_ToText() const
	{
		return (m_n_type == type_Text)? this : p_InvalidNode();
	}

	inline bool b_IsText() const
	{
		return m_n_type == type_Text;
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_ToCData() const
	 *		- returns pointer to this if this node is CData.
	 *		  otherwise returns invalid node (invalid node is 0 or dummy node,
	 *		  depending on build config)
	 */
	inline const CXMLNode *p_ToCData() const
	{
		return (m_n_type == type_CData)? this : p_InvalidNode();
	}

	inline bool b_IsCData() const
	{
		return m_n_type == type_CData;
	}

	inline bool b_IsPI() const
	{
		return m_n_type == type_ProcInstr;
	}

	inline bool b_IsDTD() const
	{
		return m_n_type == type_DocTypeDecl;
	}

	inline bool b_IsDoc() const
	{
		return m_n_type == type_Document;
	}

#ifdef __XML_NODE_USE_DUMMY

	/**
	 *	@note This function should not be called if dummy node is not enabled.
	 *		Use b_IsValidNode() instead.
	 */
	inline bool b_IsDummy() const
	{
		return m_n_type == type_DummyNode;
	}

#endif // __XML_NODE_USE_DUMMY

	inline bool b_IsXMLDecl() const
	{
		return m_n_type == type_XMLDecl;
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
	const wstring &s_InnerText() const;

	/*
	 *	inline const CXMLNode::wstring &CXMLNode::s_Name() const
	 *		- returns node name
	 */
	inline const wstring &s_Name() const
	{
		return m_s_name;
	}

	/*
	 *	inline const CXMLNode::wstring &CXMLNode::s_Value() const
	 *		- returns node value (value of text / CData nodes
	 *		  or unparsed arguments of PI node)
	 */
	inline const wstring &s_Value() const
	{
		return m_s_value;
	}

	/*
	 *	inline int CXMLNode::n_Attrib_Num() const
	 *		- returns number of node attributes
	 */
	inline size_t n_Attrib_Num() const
	{
		return m_attrib_list.size();
	}

	/*
	 *	const CXMLNode::TAttrib &CXMLNode::t_Attrib(int n_index) const
	 *		- returns attribute with zero-based index n_index
	 *		  (note this doesn't check array bounds)
	 */
	const TAttrib &t_Attrib(size_t n_index) const
	{
		return m_attrib_list[n_index];
	}

	/*
	 *	const CXMLNode::wstring *CXMLNode::p_Attrib(const char *p_s_name) const
	 *		- returns value of attribute with name p_s_name,
	 *		  or 0 in case there's no attribute with such name
	 */
	const wstring *p_Attrib(const char *p_s_name) const;

	/*
	 *	const CXMLNode::wstring *CXMLNode::p_Attrib(const wstring &r_s_name) const
	 *		- returns value of attribute with name r_s_name,
	 *		  or 0 in case there's no attribute with such name
	 */
	const wstring *p_Attrib(const wstring &r_s_name) const;

	/*
	 *	inline int CXMLNode::n_Child_Num() const
	 *		- returns number of child nodes
	 */
	inline size_t n_Child_Num() const
	{
		return m_subnode_list.size();
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_Child(int n_index) const
	 *		- returns child node with zero-based index n_index
	 *		  (note this doesn't check array bounds)
	 */
	inline const CXMLNode *p_Child(size_t n_index) const
	{
		return m_subnode_list[n_index];
	}

	/*
	 *	const CXMLNode *CXMLNode::p_ChildAt(int n_index) const
	 *		- returns child node with zero-based index n_index
	 *		- if n_index is out of array bounds, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	const CXMLNode *p_ChildAt(size_t n_index) const;

	/*
	 *	const CXMLNode *CXMLNode::p_FirstChild() const
	 *		- returns first child node
	 *		- in case node list is empty, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	const CXMLNode *p_FirstChild() const;

	/*
	 *	const CXMLNode *CXMLNode::p_LastChild() const
	 *		- returns last child node
	 *		- in case node list is empty, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	const CXMLNode *p_LastChild() const;

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindChild(const char *p_s_name) const
	 *		- returns first child node with name p_s_name
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindChild(const char *p_s_name) const
	{
		return p_BaseFind(m_subnode_list.begin(), CFindByName(p_s_name));
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindNextChild(const CXMLNode *p_prev,
	 *		const char *p_s_name) const
	 *		- returns child node with name p_s_name, coming after child node p_prev
	 *		- note name of p_prev it is not required to be r_s_name,
	 *		  it can be any of child nodes
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindNextChild(const CXMLNode *p_prev, const char *p_s_name) const
	{
		std::vector<CXMLNode*>::const_iterator p_node_it =
			std::find(m_subnode_list.begin(), m_subnode_list.end(), p_prev);
		if(p_node_it != m_subnode_list.end())
			return p_BaseFind(p_node_it + 1, CFindByName(p_s_name));
		return p_InvalidNode();
	}

	/*
	 *	inline const CXMLNode *p_FindChild(const wstring &r_s_name) const
	 *		- returns first child node with name p_s_name
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindChild(const wstring &r_s_name) const
	{
		return p_BaseFind(m_subnode_list.begin(), CFindByStlName(r_s_name));
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindNextChild(const CXMLNode *p_prev,
	 *		const wstring &r_s_name) const
	 *		- returns child node with name r_s_name, coming after child node p_prev
	 *		- note name of p_prev it is not required to be r_s_name,
	 *		  it can be any of child nodes
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindNextChild(const CXMLNode *p_prev, const wstring &r_s_name) const
	{
		std::vector<CXMLNode*>::const_iterator p_node_it =
			std::find(m_subnode_list.begin(), m_subnode_list.end(), p_prev);
		if(p_node_it != m_subnode_list.end())
			return p_BaseFind(p_node_it + 1, CFindByStlName(r_s_name));
		return p_InvalidNode();
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindChildElement(const char *p_s_name) const
	 *		- returns first child node of type type_Element with name p_s_name
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindChildElement(const char *p_s_name) const
	{
		return p_BaseFind(m_subnode_list.begin(), CFindElemByName(p_s_name));
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindNextChildElement(const CXMLNode *p_prev,
	 *		const char *p_s_name) const
	 *		- returns child node of type type_Element with name p_s_name, coming after
	 *		  child node p_prev
	 *		- note name of p_prev it is not required to be r_s_name,
	 *		  it can be any of child nodes
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindNextChildElement(const CXMLNode *p_prev, const char *p_s_name) const
	{
		std::vector<CXMLNode*>::const_iterator p_node_it =
			std::find(m_subnode_list.begin(), m_subnode_list.end(), p_prev);
		if(p_node_it != m_subnode_list.end())
			return p_BaseFind(p_node_it + 1, CFindElemByName(p_s_name));
		return p_InvalidNode();
	}

	/*
	 *	inline const CXMLNode *p_FindChildElement(const wstring &r_s_name) const
	 *		- returns first child node of type type_Element with name p_s_name
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindChildElement(const wstring &r_s_name) const
	{
		return p_BaseFind(m_subnode_list.begin(), CFindElemByStlName(r_s_name));
	}

	/*
	 *	inline const CXMLNode *CXMLNode::p_FindNextChild(const CXMLNode *p_prev,
	 *		const wstring &r_s_name) const
	 *		- returns child node with name of type type_Element r_s_name, coming
	 *		  after child node p_prev
	 *		- note name of p_prev it is not required to be r_s_name,
	 *		  it can be any of child nodes
	 *		- in case there is not node with such name, returns invalid node
	 *		  (invalid node is 0 or dummy node, depending on build config)
	 */
	inline const CXMLNode *p_FindNextChildElement(const CXMLNode *p_prev,
		const wstring &r_s_name) const
	{
		std::vector<CXMLNode*>::const_iterator p_node_it =
			std::find(m_subnode_list.begin(), m_subnode_list.end(), p_prev);
		if(p_node_it != m_subnode_list.end())
			return p_BaseFind(p_node_it + 1, CFindElemByStlName(r_s_name));
		return p_InvalidNode();
	}

	/**
	 *	@brief performs depth first search for a child element
	 *
	 *	@param[in] pred is predicate that identifies the element being searched
	 *	@param[in] n_max_depth is search depth limit (0 means there is no limit)
	 *
	 *	@return Returns pointer to a node if fount or invalid node if not found
	 *		(invalid node is 0 or dummy node, depending on build config).
	 *
	 *	@note The search needs to allocate some memory so this throws std::bad_alloc,
	 *		because there is no intuitive way of determining whether the node
	 *		was not found, or it was a low memory error.
	 */
	template <class CPredicate>
	const CXMLNode *p_FindChildElem_DFS(CPredicate pred, size_t n_max_depth = 0) const // throws(std::bad_alloc)
	{
		//try {
			if(!n_max_depth) {
				std::vector<const CXMLNode*> search_list;
				search_list.push_back(this);
				while(!search_list.empty()) {
					const CXMLNode *p_search = search_list.back();
					search_list.erase(search_list.end() - 1);
					// get a node to search

					for(size_t i = 0, n = p_search->n_Child_Num(); i < n; ++ i) {
						const CXMLNode *p_node = p_search->p_Child(i);
						if(p_node->b_IsElement() && pred(*p_node))
							return p_node;

						search_list.push_back(p_node);
					}
					// unpack the node to the list
				}
			} else {
				std::vector<std::pair<const CXMLNode*, size_t> > search_list;
				search_list.push_back(std::make_pair(this, size_t(0)));
				while(!search_list.empty()) {
					size_t n_level = search_list.back().second;
					const CXMLNode *p_search = search_list.back().first;
					search_list.erase(search_list.end() - 1);
					// get a node to search

					if(n_level < n_max_depth) {
						_ASSERTE(n_level < SIZE_MAX);
						++ n_level;
						for(size_t i = 0, n = p_search->n_Child_Num(); i < n; ++ i) {
							const CXMLNode *p_node = p_search->p_Child(i);
							if(p_node->b_IsElement() && pred(*p_node))
								return p_node;

							search_list.push_back(std::make_pair(p_node, n_level));
						}
						// 
					} else {
						for(size_t i = 0, n = p_search->n_Child_Num(); i < n; ++ i) {
							const CXMLNode *p_node = p_search->p_Child(i);
							if(p_node->b_IsElement() && pred(*p_node))
								return p_node;
						}
						// go through nodes, but do not go any deeper
					}
					// unpack the node to the list
				}
				// depth-limited search needs more memory
			}
		//	return p_root; // not found
		//} catch(std::bad_alloc&) {
			return CXMLNode::p_InvalidNode(); // not enough memory
		//}
	}

	/*
	 *	static inline const CXMLNode *CXMLNode::p_InvalidNode()
	 *		- returns invalid node, that is either 0, or dummy node,
	 *		  depending on wheter __XML_NODE_USE_DUMMY is defined
	 *		- do not use constructs such as my_node == p_InvalidNode(),
	 *		  use b_IsValidNode(my_node) instead
	 */
	static inline const CXMLNode *p_InvalidNode()
	{
#ifdef __XML_NODE_USE_DUMMY
		return p_DummyNode();
#else // __XML_NODE_USE_DUMMY
		return 0;
#endif // __XML_NODE_USE_DUMMY
	}

	/*
	 *	static inline bool b_IsValidNode(const CXMLNode *p_node)
	 *		- predicate wheter p_node is valid node
	 *		  (not 0, not dummy node)
	 */
	static inline bool b_IsValidNode(const CXMLNode *p_node)
	{
#ifdef __XML_NODE_USE_DUMMY
		return p_node != 0 && p_node->m_n_type != type_DummyNode;
#else // __XML_NODE_USE_DUMMY
		return p_node != 0;
#endif // __XML_NODE_USE_DUMMY
	}

protected:
	static inline void DeleteNode(CXMLNode *p_node);
#ifdef __XML_NODE_USE_DUMMY
	static const CXMLNode *p_DummyNode();
#endif // __XML_NODE_USE_DUMMY
	template <class CPredicate>
	const CXMLNode *p_BaseFind(_node_it p_begin_it, CPredicate pred) const
	{
		std::vector<CXMLNode*>::const_iterator p_node_it =
			std::find_if(p_begin_it, m_subnode_list.end(), pred);
		if(p_node_it != m_subnode_list.end())
			return *p_node_it;
		return p_InvalidNode();
	}
	static bool ParseLoop(CXMLNode *p_doc, CXML_EventParser &r_parser);
};

#endif // !__XML_PARSER_INCLUDED
