/*
								+----------------------------------+
								|                                  |
								|   ***  Unicode conversion  ***   |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|            UniConv.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __UNICODE_CONVERSION_INCLUDED
#define __UNICODE_CONVERSION_INCLUDED

/**
 *	@file UniConv.h
 *	@date 2008
 *	@author -tHE SWINe-
 *	@brief unicode conversion
 *
 *	@date 2009-07-09
 *
 *	fixed unicode mappings url (http://www.unicode.org/Public/MAPPINGS/)
 *
 *	added more complete list of conversions between 8-bit, UTF-8, UTF-16
 *	(LE or BE) and UTF-32 to CUnicodeConversion
 *
 *	added alias CUniConv for CUnicodeConversion
 *
 *	@date 2009-09-13
 *
 *	changed CUnicodeMapping::TCharacterMapping::n_character to unsigned (signed
 *	caused most encodings to fail working with character codes above 128,
 *	CUnicodeMapping::n_FromUnicode() and CUnicodeMapping::FromUnicode() functions
 *	were affected by this change)
 *
 *	@date 2009-10-11
 *
 *	changed type of input data from const uint8_t* to const void* in some of
 *	CUniConv routines (convenience, do not have to type-cast anymore). functionality
 *	remains unchanged.
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	@date 2010-11-05
 *
 *	Changed string size parameters type in CUniConv functions from int to size_t
 *	(64-bit compatibility).
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2012-06-20
 *
 *	Added the CUniConv::n_UTF16_Char_Size(), CUniConv::n_UTF16_Encode_Size(),
 *	CUniConv::n_UTF16_SafeEncode_Size(), CUniConv::n_UTF8_Encode_Size() and
 *	CUniConv::n_UTF8_SafeEncode_Size() functions.
 *
 *	Added the CUniConv::n_EncodeCharacter_UTF8(), CUniConv::n_EncodeCharacter_UTF8_Strict(),
 *	CUniConv::n_EncodeCharacter_UTF16_LE(), CUniConv::n_EncodeCharacter_UTF16_LE_Strict(),
 *	CUniConv::n_EncodeCharacter_UTF16_BE() and CUniConv::n_EncodeCharacter_UTF16_BE_Strict().
 *
 *	Added the CUniConv::Encode_UTF32() and CUniConv::Decode_UTF32() functions.
 *
 *	Added byte order mark functions.
 *
 *	Wrote function documentation comments.
 *
 *	@todo this is mostly untested (especially the code regarding BOMs) - test it, compile it on linux
 *
 *	@date 2013-05-04
 *
 *	Added \#ifdef arround _strdup() function to make it compile on non-MSVC compilers.
 *
 *	@date 2014-08-15
 *
 *	Added wchar_t functions.
 *
 */

#include "Integer.h"
#include "StlUtils.h"
#include <stdio.h>
#include <string.h>
#include <string>
#include <wchar.h>

class CUnicodeMapping;

/**
 *	@brief implementation of conversion between different unicode representations,
 *		specifically between UTF-8, UTF-16 and UTF-32 and between 8-bit charsets
 *	@note The string lengths for UTF-8 and UTF-16 are in bytes, as the
 *		encodings have variable code size. UTF-32 string lengths are
 *		in *characters*, every UTF-32 character is exactly four bytes.
 *		This is also true for 8-bit charsets, but there length in characters
 *		is equal to length in bytes and so the terms are interchangable.
 *		In the end, this only concerns Decode_UTF32(), UTF32_to_UTF8() and
 *		UTF32_to_UTF16(), as these are the only functions to get UTF-32 string
 *		length as input.
 */
class CUniConv {
public:
	typedef int char32; /**< @brief wide (32-bit) char type */
	typedef std::basic_string<char32> wstring32; /**< @brief wide (32-bit) string type */

	typedef unsigned short char16; /**< @brief wide (16-bit) char type */
	typedef std::basic_string<char16> wstring16; /**< @brief wide (16-bit) string type */

	//		--- wchar_t functions ---

	/**
	 *	@brief converts system default wide character string (wchar_t) to UTF-8
	 *
	 *	@param[in] p_data is wchar_t string
	 *	@param[in] n_length is length of the input string in characters, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-8 encoded string from p_data
	 *	@param[in] b_use_utf8_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_wchar_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool Wide_to_UTF8(const wchar_t *p_data, size_t n_length, std::string &r_s_string,
		bool b_use_utf8_bom, bool b_allow_wchar_bom = false);

	/**
	 *	@brief converts system default wide character string (wchar_t) to UTF-16
	 *
	 *	@param[in] p_data is wchar_t string
	 *	@param[in] n_length is length of the input string in characters, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-16 encoded string from p_data
	 *	@param[in] b_use_utf16_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_wchar_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_utf16_little_endian is endianness flag (if set, UTF-16 LE is encoded,
	 *		otherwise UTF-16 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool Wide_to_UTF16(const wchar_t *p_data, size_t n_length, wstring16 &r_s_string,
		bool b_use_utf16_bom, bool b_allow_wchar_bom = false, bool b_utf16_little_endian = true);

	/**
	 *	@brief converts system default wide character string (wchar_t) to UTF-32
	 *
	 *	@param[in] p_data is wchar_t string
	 *	@param[in] n_length is length of the input string in characters, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-32 encoded string from p_data
	 *	@param[in] b_use_utf32_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_wchar_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_utf32_little_endian is endianness flag (if set, UTF-32 LE is encoded,
	 *		otherwise UTF-32 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool Wide_to_UTF32(const wchar_t *p_data, size_t n_length, wstring32 &r_s_string,
		bool b_use_utf32_bom, bool b_allow_wchar_bom = false, bool b_utf32_little_endian = true);

	/**
	 *	@brief converts UTF-8 to system default wide character string (wchar_t)
	 *
	 *	@param[in] p_data is UTF-8 string
	 *	@param[in] n_size is size of the input string in bytes, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with wchar_t string from p_data
	 *	@param[in] b_use_wchar_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_utf8_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool UTF8_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
		bool b_use_wchar_bom, bool b_allow_utf8_bom = false);

	/**
	 *	@brief converts UTF-16 to system default wide character string (wchar_t)
	 *
	 *	@param[in] p_data is UTF-16 string
	 *	@param[in] n_size is size of the input string in bytes, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with wchar_t string from p_data
	 *	@param[in] b_use_wchar_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_utf16_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf16_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-16 LE is expected, otherwise UTF-16 BE is expected)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool UTF16_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
		bool b_use_wchar_bom, bool b_allow_utf16_bom = false, bool b_expect_utf16_little_endian = true);

	/**
	 *	@brief converts UTF-32 to system default wide character string (wchar_t)
	 *
	 *	@param[in] p_data is UTF-32 string
	 *	@param[in] n_size is size of the input string in bytes, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with wchar_t string from p_data
	 *	@param[in] b_use_wchar_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_utf32_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf32_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-32 LE is expected, otherwise UTF-32 BE is expected)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 *	@note The endianness of wchar_t is assumed to be the same as the machine endianness.
	 */
	static bool UTF32_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
		bool b_use_wchar_bom, bool b_allow_utf32_bom = false, bool b_expect_utf32_little_endian = true);

	//		--- UTF-32 functions ---

	/**
	 *	@brief gets UTF-32 LE byte order mark
	 *	@return Returns array of 4 bytes, containing the UTF-32 LE byte order mark.
	 */
	static inline const char *p_UTF32_LE_ByteOrderMark()
	{
		static const char p_bom[] = {(char)0xff, (char)0xfe, (char)0x00, (char)0x00}; // fixme
		return p_bom;
	}

	/**
	 *	@brief gets UTF-32 BE byte order mark
	 *	@return Returns array of 4 bytes, containing the UTF-32 BE byte order mark.
	 */
	static inline const char *p_UTF32_BE_ByteOrderMark()
	{
		static const char p_bom[] = {(char)0x00, (char)0x00, (char)0xfe, (char)0xff}; // fixme
		return p_bom;
	}

	/**
	 *	@brief decodes UTF-32 string to 8-bit string (codes above 255 are replaced by '?')
	 *
	 *	@param[in] p_data is UTF-32 string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *		(codes above 255 are replaced by '?')
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-32 LE is expected, otherwise UTF-32 BE is expected)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool Decode_UTF32(const int *p_data, size_t n_length,
		std::string &r_s_string, bool b_allow_bom, bool b_expect_little_endian = true);

	/**
	 *	@brief decodes UTF-32 string to 8-bit string using a mapping table
	 *
	 *	@param[in] p_data is UTF-32 string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *	@param[in] r_map is unicode mapping table to be used for the conversion
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-32 LE is expected, otherwise UTF-32 BE is expected)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool Decode_UTF32(const int *p_data, size_t n_length,
		std::string &r_s_string, const CUnicodeMapping &r_map,
		bool b_allow_bom, bool b_expect_little_endian = true);

	/**
	 *	@brief encodes generic 8-bit encoded characters as UTF-32
	 *
	 *	@param[in] p_data is generic 8-bit string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] p_mapping_table is table with 256 entries for each 8-bit code, containing
	 *		corresponding UTF-32 character, or negative number for undefined characters
	 *		(note entry with index 0 is always ignored, 8-bit char 0 is terminating zero)
	 *	@param[out] r_s_string is overwritten with UTF-32 string from p_data
	 *	@param[in] b_use_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_little_endian is endianness flag (if set, UTF-32 LE is encoded,
	 *		otherwise UTF-32 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool Encode_UTF32(const char *p_data, size_t n_length,
		const int *p_mapping_table, wstring32 &r_s_string,
		bool b_use_bom, bool b_little_endian = true);

	/**
	 *	@brief encodes plain unicode characters (UTF-32) as UTF-8
	 *
	 *	@param[in] p_data is UTF-32 string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] b_allow_utf32_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf32_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-32 LE is expected, otherwise UTF-32 BE is expected)
	 *	@param[out] r_s_string is overwritten with UTF-8 encoded string from p_data
	 *	@param[in] b_use_utf8_bom is byte order marker flag (if set, BOM is written in the output)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool UTF32_to_UTF8(const int *p_data, size_t n_length,
		bool b_allow_utf32_bom, bool b_expect_utf32_little_endian,
		std::string &r_s_string, bool b_use_utf8_bom);

	/**
	 *	@brief encodes plain unicode characters (UTF-32) as UTF-16
	 *
	 *	@param[in] p_data is UTF-32 string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] b_allow_utf32_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf32_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-32 LE is expected, otherwise UTF-32 BE is expected)
	 *	@param[out] r_s_string is overwritten with UTF-16 encoded string from p_data
	 *	@param[in] b_use_utf16_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_utf16_little_endian is endianness flag (if set, UTF-16 LE is encoded,
	 *		otherwise UTF-16 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool UTF32_to_UTF16(const int *p_data, size_t n_length,
		bool b_allow_utf32_bom, bool b_expect_utf32_little_endian, wstring16 &r_s_string,
		bool b_use_utf16_bom = false, bool b_utf16_little_endian = true);

	/**
	 *	@brief swaps byte order in a 32-bit double-word
	 *		(performs conversion between big and little endian for UTF-32)
	 *	@param[in] n_code is input value
	 *	@return Returns n_code with byte order swapped.
	 */
	static inline uint32_t n_ByteSwap32(uint32_t n_code)
	{
		return (n_code >> 24) | ((n_code >> 8) & 0xff00) |
			((n_code & 0xff00) << 8) | (n_code << 24);
	}

	//		--- UTF-16 functions ---

	/**
	 *	@brief gets UTF-16 LE byte order mark
	 *	@return Returns array of 2 bytes, containing the UTF-16 LE byte order mark.
	 */
	static inline const char *p_UTF16_LE_ByteOrderMark()
	{
		static const char p_bom[] = {(char)0xff, (char)0xfe}; // fixme
		return p_bom;
	}

	/**
	 *	@brief gets UTF-16 BE byte order mark
	 *	@return Returns array of 2 bytes, containing the UTF-16 BE byte order mark.
	 */
	static inline const char *p_UTF16_BE_ByteOrderMark()
	{
		static const char p_bom[] = {(char)0xfe, (char)0xff}; // fixme
		return p_bom;
	}

	/**
	 *	@brief converts UTF-16 encoded string to us-ascii (codes above 255 are replaced by '?')
	 *
	 *	@param[in] p_data is UTF-16 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *		(codes above 255 are replaced by '?')
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-16 LE is expected, otherwise UTF-16 BE is expected)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_Decode_UTF16(const void *p_data, size_t n_size, std::string &r_s_string,
		bool b_allow_bom, bool b_expect_little_endian = true);

	/**
	 *	@brief converts UTF-16 encoded string to 8-bit string using a mapping table
	 *
	 *	@param[in] p_data is UTF-16 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *	@param[in] r_map is unicode mapping table to be used for the conversion
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-16 LE is expected, otherwise UTF-16 BE is expected)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note In case mapping of unicode to 8-bit charset doesn't exist, function behavior
	 *		depends on value of substitute character set in r_map. In case it's negative
	 *		(default), the function fails. Otherwise the replacement character is used.
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_Decode_UTF16(const void *p_data, size_t n_size, std::string &r_s_string,
		const CUnicodeMapping &r_map, bool b_allow_bom, bool b_expect_little_endian = true);

	/**
	 *	@brief encodes generic 8-bit encoded characters as UTF-16
	 *
	 *	@param[in] p_data is generic 8-bit string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] p_mapping_table is table with 256 entries for each 8-bit code, containing
	 *		corresponding UTF-32 character, or negative number for undefined characters
	 *		(note entry with index 0 is always ignored, 8-bit char 0 is terminating zero)
	 *	@param[out] r_s_string is overwritten with UTF-16 string from p_data
	 *	@param[in] b_use_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_little_endian is endianness flag (if set, UTF-16 LE is encoded,
	 *		otherwise UTF-16 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool Encode_UTF16(const char *p_data, size_t n_length,
		const int *p_mapping_table, wstring16 &r_s_string,
		bool b_use_bom = false, bool b_little_endian = true);

	/**
	 *	@brief converts UTF-16 encoded string to UTF-32
	 *
	 *	@param[in] p_data is UTF-16 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-32 string from p_data
	 *	@param[in] b_use_utf32_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_utf32_little_endian is endianness flag (if set, UTF-32 LE is encoded,
	 *		otherwise UTF-32 BE is encoded)
	 *	@param[in] b_allow_utf16_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf16_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-16 LE is expected, otherwise UTF-16 BE is expected)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_UTF16_to_UTF32(const void *p_data, size_t n_size,
		wstring32 &r_s_string, bool b_allow_utf16_bom,
		bool b_expect_utf16_little_endian = true,
		bool b_use_utf32_bom = false, bool b_utf32_little_endian = true);

	/**
	 *	@brief converts UTF-16 encoded string to UTF-8
	 *
	 *	@param[in] p_data is UTF-16 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-8 string from p_data
	 *	@param[in] b_use_utf8_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_allow_utf16_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_expect_utf16_little_endian is default endianness flag if BOM is not present
	 *		(if set, UTF-16 LE is expected, otherwise UTF-16 BE is expected)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_UTF16_to_UTF8(const void *p_data, size_t n_size, std::string &r_s_string,
		bool b_use_utf8_bom, bool b_allow_utf16_bom,
		bool b_expect_utf16_little_endian = true);

	/**
	 *	@brief calculates how many bytes are required to encode character using UTF-16 (LE or BE)
	 *	@param[in] n_char is character to encode (must be in the 0 - 0x10ffff range)
	 *	@return Returns space required to encode the given character, in bytes (2 or 4).
	 */
	static int n_UTF16_Encode_Size(int n_char);

	/**
	 *	@brief calculates how many bytes are required to encode character using UTF-16 (LE or BE)
	 *	@param[in] n_char is character to encode
	 *	@return Returns space required to encode the given character, in bytes (2 or 4),
	 *		returns -1 in case the character is out of range.
	 */
	static int n_UTF16_SafeEncode_Size(int n_char);

	/**
	 *	@brief calculates size of UTF-16 LE character based on it's first two bytes
	 *
	 *	@param[in] n_first_byte is the first byte from the input stream
	 *	@param[in] n_second_byte is the second byte from the input stream
	 *
	 *	@return Returns size of the character in bytes (2 or 4 for surrogate pair),
	 *		returns -1 on failure (lonely low surrogate).
	 *
	 *	@note Actually only the second byte is required, but that should be
	 *		optimized-away in inline expansion of the function.
	 */
	static int n_UTF16_LE_Char_Size(uint8_t UNUSED(n_first_byte), uint8_t n_second_byte);

	/**
	 *	@brief calculates size of UTF-16 BE character based on it's first two bytes
	 *
	 *	@param[in] n_first_byte is the first byte from the input stream
	 *	@param[in] n_second_byte is the second byte from the input stream
	 *
	 *	@return Returns size of the character in bytes (2 or 4 for surrogate pair),
	 *		returns -1 on failure (lonely low surrogate).
	 *
	 *	@note Actually only the first byte is required, but that should be
	 *		optimized-away in inline expansion of the function.
	 */
	static int n_UTF16_BE_Char_Size(uint8_t n_first_byte, uint8_t UNUSED(n_second_byte));

	/**
	 *	@brief calculates size of UTF-16 character, based on it's first two bytes
	 *	@param[in] n_first_word is the word, composed of first two bytes of the character,
	 *		in the machine order (if machine is LE and encoding is UTF-16 BE, the bytes
	 *		need to be swapped)
	 *	@return Returns character size in bytes (either 2 or 4 (surrogate pair)),
	 *		or -1 on failure (low surrogate).
	 */
	static int n_UTF16_Char_Size(uint16_t n_first_word);

	/**
	 *	@brief decodes a single UTF-16 (little endian) character
	 *
	 *	@param[in] p_data is buffer with UTF-16 LE data
	 *	@param[in] n_size is amount of data in p_data, in bytes
	 *	@param[out] r_n_read is number of bytes read from input buffer
	 *		(written upon function return)
	 *
	 *	@return Returns character code (UTF-32) on success, -1 on failure.
	 *
	 *	@note In case byte order mark is encountered, it is interpreted
	 *		as ordinary character code.
	 */
	static int n_UTF16_LE_Code(const void *p_data, size_t n_size, int &r_n_read);

	/**
	 *	@brief decodes a single UTF-16 (big endian) character
	 *
	 *	@param[in] p_data is buffer with UTF-16 BE data
	 *	@param[in] n_size is amount of data in p_data, in bytes
	 *	@param[out] r_n_read is number of bytes read from input buffer
	 *		(written upon function return)
	 *
	 *	@return Returns character code (UTF-32) on success, -1 on failure.
	 *
	 *	@note In case byte order mark is encountered, it is interpreted
	 *		as ordinary character code.
	 */
	static int n_UTF16_BE_Code(const void *p_data, size_t n_size, int &r_n_read);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-16 LE
	 *
	 *	@param[out] p_dest is destination for UTF-16 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest, in bytes
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns number of bytes written to p_dest, or 0 on failure
	 *		(not enough space, forbidden character or too high character).
	 */
	static int n_EncodeCharacter_UTF16_LE(void *p_dest, size_t n_space, int n_character);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-16 BE
	 *
	 *	@param[out] p_dest is destination for UTF-16 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest, in bytes
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns number of bytes written to p_dest, or 0 on failure
	 *		(not enough space, forbidden character or too high character).
	 */
	static int n_EncodeCharacter_UTF16_BE(void *p_dest, size_t n_space, int n_character);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-16 LE
	 *
	 *	@param[out] p_dest is destination for UTF-16 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest and also amount of data
	 *		the character is supposed to occupy, in bytes
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns true on success, false on failure (not enough / too much
	 *		space, forbidden character or too high character).
	 */
	static bool EncodeCharacter_UTF16_LE_Strict(void *p_dest, size_t n_space, int n_character);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-16 BE
	 *
	 *	@param[out] p_dest is destination for UTF-16 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest and also amount of data
	 *		the character is supposed to occupy, in bytes
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns true on success, false on failure (not enough / too much
	 *		space, forbidden character or too high character).
	 */
	static bool EncodeCharacter_UTF16_BE_Strict(void *p_dest, size_t n_space, int n_character);

	/**
	 *	@brief swaps high and low byte in a 16-bit word
	 *		(performs conversion between big and little endian for UTF-16)
	 *	@param[in] n_code is input value
	 *	@return Returns n_code with high and low byte swapped.
	 */
	static inline uint16_t n_HiLoSwap(uint16_t n_code)
	{
		return (n_code >> 8) | (n_code << 8); // xchg al, ah
	}

	//		--- UTF-8 functions ---

	/**
	 *	@brief gets UTF-8 byte order mark
	 *	@return Returns array of 3 bytes, containing the UTF-8 byte order mark.
	 */
	static inline const char *p_UTF8_ByteOrderMark()
	{
		static const char p_bom[] = {(char)0xef, (char)0xbb, (char)0xbf}; // fixme
		return p_bom;
	}

	/**
	 *	@brief converts UTF-8 encoded string to us-ascii (codes above 255 are replaced by '?')
	 *
	 *	@param[in] p_data is UTF-8 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *		(codes above 255 are replaced by '?')
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_Decode_UTF8(const void *p_data, size_t n_size,
		std::string &r_s_string, bool b_allow_bom);

	/**
	 *	@brief converts UTF-8 encoded string to 8-bit string using a mapping table
	 *
	 *	@param[in] p_data is UTF-8 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with 8-bit string from p_data
	 *	@param[in] r_map is unicode mapping table to be used for the conversion
	 *	@param[in] b_allow_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note In case mapping of unicode to 8-bit charset doesn't exist, function behavior
	 *		depends on value of substitute character set in r_map. In case it's negative
	 *		(default), the function fails. Otherwise the replacement character is used.
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_Decode_UTF8(const void *p_data, size_t n_size,
		std::string &r_s_string, const CUnicodeMapping &r_map, bool b_allow_bom);

	/**
	 *	@brief encodes generic 8-bit encoded characters as UTF-8
	 *
	 *	@param[in] p_data is generic 8-bit string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] p_mapping_table is table with 256 entries for each 8-bit code, containing
	 *		corresponding UTF-32 character, or negative number for undefined characters
	 *		(note entry with index 0 is always ignored, 8-bit char 0 is terminating zero)
	 *	@param[out] r_s_string is overwritten with UTF-8 string from p_data
	 *	@param[in] b_use_bom is byte order marker flag (if set, BOM is written in the output)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static bool Encode_UTF8(const char *p_data, size_t n_size,
		const int *p_mapping_table, std::string &r_s_string, bool b_use_bom);

	/**
	 *	@brief converts UTF-8 encoded string to UTF-32
	 *
	 *	@param[in] p_data is UTF-8 string
	 *	@param[in] n_size is size of the input string (in bytes), or -1 in case it is null-terminated
	 *	@param[out] r_s_string is overwritten with UTF-32 string from p_data
	 *	@param[in] b_allow_utf8_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[in] b_use_utf32_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_utf32_little_endian is endianness flag (if set, UTF-32 LE is encoded,
	 *		otherwise UTF-32 BE is encoded)
	 *
	 *	@return Returns number of bytes read from the buffer (including the two bytes
	 *		for terminating null), or -1 on failure.
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_UTF8_to_UTF32(const void *p_data,
		size_t n_size, wstring32 &r_s_string, bool b_allow_utf8_bom,
		bool b_use_utf32_bom = false, bool b_utf32_little_endian = true);

	/**
	 *	@brief converts UTF-8 encoded string to UTF-16
	 *
	 *	@param[in] p_data is UTF-8 string
	 *	@param[in] n_length is length of the input string, or -1 in case it is null-terminated
	 *	@param[in] b_allow_utf8_bom is byte order mark tolerance flag (if set, BOM is read and interpreted,
	 *		otherwise if BOM is encountered, it is interpreted as an ordinary character)
	 *	@param[out] r_s_string is overwritten with UTF-16 encoded string from p_data
	 *	@param[in] b_use_utf16_bom is byte order marker flag (if set, BOM is written in the output)
	 *	@param[in] b_utf16_little_endian is endianness flag (if set, UTF-16 LE is encoded,
	 *		otherwise UTF-16 BE is encoded)
	 *
	 *	@return Returns true on success, false on failure (not enough memory, or invalid chars).
	 *
	 *	@note It's possible to call with n_size = -1 in case p_data contains
	 *		null-terminated string (the loop will break after decrementing size down
	 *		to zero (2^32 - 1 chars for -1), or when encountered zero character).
	 */
	static int n_UTF8_to_UTF16(const void *p_data, size_t n_size,
		wstring16 &r_s_string, bool b_allow_utf8_bom,
		bool b_include_utf16_bom = false, bool b_utf16_little_endian = true);

	/**
	 *	@brief calculates how many bytes are required to encode character using UTF-8
	 *	@param[in] n_char is character to encode (must be in the 0 - 0x10ffff range)
	 *	@return Returns space required to encode the given character, in bytes (1 to 4).
	 */
	static int n_UTF8_Encode_Size(int n_char);

	/**
	 *	@brief calculates how many bytes are required to encode character using UTF-8
	 *	@param[in] n_char is character to encode
	 *	@return Returns space required to encode the given character, in bytes (1 to 4),
	 *		returns -1 in case the character is out of range.
	 */
	static int n_UTF8_SafeEncode_Size(int n_char);

	/**
	 *	@brief calculates size of UTF-8 character based on it's first byte
	 *	@param[in] n_first_byte is the first byte of the character
	 *	@return Returns size in bytes (includes the first byte, values range 1 to 4),
	 *		or -1 on failure (invalid UTF-8 character).
	 */
	static int n_UTF8_Char_Size(uint8_t n_first_byte);

	/**
	 *	@brief decodes a single UTF-8 character
	 *
	 *	@param[in] p_data is buffer with UTF-8 data
	 *	@param[in] n_size is size of the data in the buffer, in bytes
	 *	@param[out] r_n_read is written number of bytes read from input
	 *		buffer upon function return
	 *
	 *	@return Returns character code (UTF-32) on success, -1 on failure.
	 *
	 *	@note This doesn't allow UTF-16 surrogates (character range 0xd800 to 0xdfff)
	 *		or characters above 0x10ffff (returns -1 instead).
	 */
	static int n_UTF8_Code(const void *p_data, size_t n_size, int &r_n_read);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-8
	 *
	 *	@param[out] p_dest is destination for UTF-8 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest, in bytes (equals characters)
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns number of bytes (equals characters) written to p_dest,
	 *		or 0 on failure (not enough space, forbidden character or too high character).
	 */
	static int n_EncodeCharacter_UTF8(char *p_dest, size_t n_space, int n_character);

	/**
	 *	@brief encodes a single UTF-32 character as UTF-8
	 *
	 *	@param[out] p_dest is destination for UTF-8 data (caller-allocated)
	 *	@param[in] n_space is free space in p_dest and also amount of data
	 *		the character is supposed to occupy, in bytes (equals characters)
	 *	@param[in] n_character is the character to be encoded
	 *
	 *	@return Returns true on success, false on failure (not enough / too much
	 *		space, forbidden character or too high character).
	 */
	static bool EncodeCharacter_UTF8_Strict(char *p_dest, size_t n_space, int n_character);
};

/**
 *	@brief typedef to retain backward compatibility
 *	@deprecated Shouldn't be used in new software.
 */
typedef CUniConv CUnicodeConversion;

//#include "Dir.h" // not required?

/**
 *	@brief unicode to 8-bit charset mapping table
 */
class CUnicodeMapping {
public:
	/**
	 *	@brief character name structure
	 */
	struct TCharacterName {
		uint8_t n_char8; /**< @brief character code in 8-bit charset */
		int n_unicode; /**< @brief UTF-32 character code */
		const char *p_s_name; /**< @brief name of the character (e.g. "LATIN CAPITAL LETTER A") */
	};

	/**
	 *	@brief parser for unicode mapping table
	 */
	class CUnicodeMappingTable {
	public:
		/**
		 *	@brief constants, stored as enum
		 */
		enum {
			max_Table_Size = 256 /**< @brief maximum table size */
		};

	protected:
		TCharacterName m_p_map[max_Table_Size];
		size_t m_n_map_size; // up to max_Table_Size
		std::string m_s_name, m_s_unicode_ver, m_s_version,
			m_s_format, m_s_date, m_s_author;
		std::string m_s_notice;

	public:
		/**
		 *	@brief default constructor; initializes an empty table
		 */
		CUnicodeMappingTable();

		/**
		 *	@brief destructor; deletes the table
		 */
		~CUnicodeMappingTable();

		/**
		 *	@brief gets table name
		 *	@return Returns table name, e.g. "ISO/IEC 8859-1:1998 to Unicode".
		 */
		const std::string &s_Name() const;

		/**
		 *	@brief gets unicode version
		 *	@return Returns unicode version, e.g. "3.2".
		 */
		const std::string &s_Unicode_Version() const;

		/**
		 *	@brief gets table version
		 *	@return Returns table version, e.g. "1.0".
		 */
		const std::string &s_Table_Version() const;

		/**
		 *	@brief gets table format
		 *	@return Returns table format, e.g. "Format A".
		 */
		const std::string &s_Format() const;

		/**
		 *	@brief gets table modification date
		 *	@return Returns table modification date, e.g. "2003 April 8".
		 */
		const std::string &s_Date() const;

		/**
		 *	@brief gets the first author of the table
		 *	@return Returns the first author of the table, e.g. "Markus Kuhn <http://www.cl.cam.ac.uk/~mgk25/>".
		 */
		const std::string &s_First_Author() const;

		/**
		 *	@brief gets table notice text
		 *	@return Returns table notice, a full copy of the commented text at the beginning of the table.
		 */
		const std::string &s_Notice() const;

		/**
		 *	@brief loads table from a file
		 *	@param[in] p_s_filename is null-terminated string, containing table file name
		 *	@return Returns true on success, false on failure.
		 */
		bool Load(const char *p_s_filename);

		/**
		 *	@brief gets table size
		 *	@return Returns table size, in characters.
		 */
		inline size_t n_Table_Size() const
		{
			return m_n_map_size;
		}

		/**
		 *	@brief gets table
		 *	@return Returns const pointer to the table entries.
		 */
		inline const TCharacterName *p_Table() const
		{
			return m_p_map;
		}

	private:
		inline CUnicodeMappingTable(const CUnicodeMappingTable &r_tab); // do not copy
		inline CUnicodeMappingTable &operator =(const CUnicodeMappingTable &r_tab); // do not copy
	};

protected:
	int m_p_mapping[256]; // for translation to unicode (UTF-32) (plain array reference)

	struct TCharacterMapping {
		unsigned char n_character;
		int n_unicode; // UTF-32
		inline operator int() const { return n_unicode; }
	} m_p_inverse_map[256]; // for translation from unicode (using binary search)
	int m_n_inverse_map_size;

	int m_n_subst_char;

public:
	/**
	 *	@brief default constructor; loads 8-bit charset mapping table from a specified file
	 *
	 *	@param[in] p_s_filename is a file with unicode mapping table (such as the files at
	 *		http://www.unicode.org/Public/MAPPINGS/, Table format: Format A)
	 *	@param[in] b_avoid_accents is accent stripping flag
	 *
	 *	@note If b_avoid_accents is set, latin accent characters are replaced by
	 *		ordinary ones. This functionality relies on comments in the file, such as:
	 *		0xC1	0x00C1	# LATIN CAPITAL LETTER A WITH ACUTE
	 *		(then the unicode character 0x00C1 will be replaced with 'A').
	 *		Also note this only affects conversion of 8-bit strings from unicode, not to unicode.
	 *	@note It's recommended to call b_Status() to see if constructor succeeded.
	 */
	CUnicodeMapping(const char *p_s_filename, bool b_avoid_accents = false);

	/**
	 *	@brief constructor; initializes the mapper from a raw table
	 *
	 *	@param[in] p_table is pointer to a table of 8-bit charset to unicode mappings
	 *	@param[in] n_table_size is table size (p_table must be allocated to n_table_size entries)
	 *	@param[in] b_avoid_accents is accent stripping flag
	 *
	 *	@note If b_avoid_accents is set, latin accent characters are replaced by
	 *		ordinary ones. This functionality relies on comments in the file, such as:
	 *		0xC1	0x00C1	# LATIN CAPITAL LETTER A WITH ACUTE
	 *		(then the unicode character 0x00C1 will be replaced with 'A').
	 *		Also note this only affects conversion of 8-bit strings from unicode, not to unicode.
	 *	@note It's recommended to call b_Status() to see if constructor succeeded.
	 */
	CUnicodeMapping(const TCharacterName *p_table, size_t n_table_size, bool b_avoid_accents = false);

//#if defined(__8BIT_CHARSETS_TO_UNICODE_CONVERSION_TABLES_INCLUDED) && !defined(__UNI_TABLES_CPP)

	/**
	 *	@brief constructor; loads a mapping table based on charset name
	 *
	 *	@param[in] p_s_charset_name is a charset name (such as "iso-8859-1")
	 *	@param[in] b_avoid_accents is accent stripping flag
	 *
	 *	@note If b_avoid_accents is set, latin accent characters are replaced by
	 *		ordinary ones. This functionality relies on comments in the file, such as:
	 *		0xC1	0x00C1	# LATIN CAPITAL LETTER A WITH ACUTE
	 *		(then the unicode character 0x00C1 will be replaced with 'A').
	 *		Also note this only affects conversion of 8-bit strings from unicode, not to unicode.
	 *	@note It's recommended to call b_Status() to see if constructor succeeded.
	 *	@note The body of this function is in UniTables.h.
	 */
	inline CUnicodeMapping(const char *p_s_charset_name, bool &r_b_charset_found,
		bool b_avoid_accents = false);

	// if getting unresolved symbol on link, don't forget to include UniTables.h !!

//#endif // __8BIT_CHARSETS_TO_UNICODE_CONVERSION_TABLES_INCLUDED && !__UNI_TABLES_CPP

	/**
	 *	@brief gets table status
	 *	@return Returns true if constructor succeeded, otherwise returns false.
	 *	@note The functions below are designed to work, even if constructor
	 *		failed (will not cause access violation / etc.).
	 */
	bool b_Status() const;

	/**
	 *	@brief translates an unicode character to 8-bit charset
	 *	@param[in] n_unicode is unicode character (UTF-32)
	 *	@return Returns 8-bit representation of the given unicode character.
	 *	@note In case the given character cannot be represented, a substitute character
	 *		is used instead (default -1, can be set using n_Set_SubsituteChar()).
	 */
	int n_FromUnicode(int n_unicode) const;

	/**
	 *	@brief translates an unicode character to 8-bit charset
	 *
	 *	@param[in] n_unicode is unicode character (UTF-32)
	 *	@param[in] n_substitute is substitute character (8-bit)
	 *
	 *	@return Returns 8-bit representation of the given unicode character.
	 *	@note In case the given character cannot be represented, the specified
	 *		substitute character is used instead.
	 */
	int n_FromUnicode(int n_unicode, int n_substitute) const;

	/**
	 *	@brief sets substitute character for conversion from unicode to the 8-bit charset
	 *
	 *	@param[in] n_substitute is substitute character (8-bit)
	 *
	 *	@return Returns the former substitute character.
	 *
	 *	@note Setting -1 as a substitute character causes conversion routines to fail
	 *		when there's no conversion for a particular character (default).
	 *	@note setting '?' as a substitute character makes conversion routines
	 *		never fail, they just return strings with question marks, insead of
	 *		characters which can't be represented in a given 8-bit charset.
	 */
	int n_Set_SubsituteChar(int n_substitute);

	/**
	 *	@brief gets the current substitute character
	 *	@return Returns the current substitute character (8-bit).
	 */
	int n_SubsituteChar() const;

	/**
	 *	@brief translates character from an 8-bit charset to unicode (UTF-32)
	 *	@param[in] n_character is a character to be encoded
	 *	@return Returns unicode (UTF-32) for the specified character, or -1 on error
	 *		(character undefined either in 8-bit charset, or in unicode).
	 */
	inline int n_ToUnicode(char n_character) const
	{
		_ASSERTE(sizeof(char) == 1); // make sure array won't overflow
		return m_p_mapping[(unsigned char)(n_character)];
	}

	/**
	 *	@brief translates unicode (UTF-32) string to 8-bit charset string
	 *
	 *	@param[out] r_s_dest is output string (filled with translated string in 8-bit charset)
	 *	@param[in] r_s_string is unicode string to be translated
	 *	@param[in] n_substitute is substitute character (8-bit; in case a given character
	 *		cannot be represented, n_substitute is used instead)
	 *
	 *	@return Returns true on success, false on failure (not enough memory).
	 */
	bool FromUnicode(std::string &r_s_dest,
		const std::basic_string<int> &r_s_string, char n_substitute = '?') const;

	/**
	 *	@brief translates a 8-bit charset string to unicode (UTF-32) string
	 *
	 *	@param[out] r_s_dest is output string (filled with translated string in UTF-32)
	 *	@param[in] r_s_string is 8-bit string to be translated
	 *
	 *	@return Returns true on success, false on failure (in case a given character
	 *		cannot be represented, the function fails).
	 */
	bool ToUnicode(std::basic_string<int> &r_s_dest, std::string &r_s_string);

	/**
	 *	@brief gets the mapping table
	 *	@return Returns a const pointer to the mapping table (256 entries, each containing
	 *		unicode for the corresponding 8-bit character, or -1 in case character can't
	 *		be represented with unicode / is not defined in the set).
	 *	@note This can be used in CUniConv::Encode_UTF8 or CUniConv::Encode_UTF16.
	 */
	inline const int *p_Get_Mapping() const
	{
		return m_p_mapping;
	}

protected:
	bool FromTable(const TCharacterName *p_table, size_t n_table_size, bool b_avoid_accents);
	static inline bool b_HigherUnicode(const TCharacterMapping &r_t_a,
		int n_unicode);
	static inline bool b_SmallerUnicode(const TCharacterMapping &r_t_a,
		const TCharacterMapping &r_t_b);
	static bool GetLine(std::string &r_s_line, FILE *p_fr);
	static bool Parse_LatinCharacterName(const char *p_s_line,
		char &r_n_char_name, bool &r_b_capital);
};

#endif // !__UNICODE_CONVERSION_INCLUDED
