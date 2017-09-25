/*
								+----------------------------------+
								|                                  |
								|   ***  Unicode conversion  ***   |
								|                                  |
								|   Copyright © -tHE SWINe- 2008   |
								|                                  |
								|           UniConv.cpp            |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file UniConv.cpp
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
 */

#include "NewFix.h"
#include "CallStack.h"
#include <stdio.h>
#include <string>
#include <algorithm>
#include "Integer.h"
#include "StlUtils.h"
#include "MinMax.h"
#include "UniConv.h"

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/*
 *								=== CUniConv ===
 */

//		--- wchar_t functions ---

bool CUniConv::Wide_to_UTF8(const wchar_t *p_data, size_t n_length, std::string &r_s_string,
	bool b_use_utf8_bom, bool b_allow_wchar_bom /*= false*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		return UTF32_to_UTF8((const int*)p_data, n_length, b_allow_wchar_bom,
			b_is_little_endian, r_s_string, b_use_utf8_bom);
	case 2:
		return n_UTF16_to_UTF8(p_data, n_length * 2, r_s_string, b_use_utf8_bom,
			b_allow_wchar_bom, b_is_little_endian) != -1;
	case 1:
		if(n_length == size_t(-1))
			return stl_ut::AssignCStr(r_s_string, (const char*)p_data); // is null terminated
		if(!stl_ut::Reserve_N(r_s_string, n_length))
			return false;
		r_s_string.insert(r_s_string.begin(), p_data, p_data + n_length);
		return true;
	default:
		return false; // not sure what wchar_t is
	}
}

bool CUniConv::Wide_to_UTF16(const wchar_t *p_data, size_t n_length, wstring16 &r_s_string,
	bool b_use_utf16_bom, bool b_allow_wchar_bom /*= false*/, bool b_utf16_little_endian /*= true*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		return UTF32_to_UTF16((const int*)p_data, n_length, b_allow_wchar_bom,
			b_is_little_endian, r_s_string, b_use_utf16_bom, b_utf16_little_endian);
	case 2:
		if(b_utf16_little_endian != b_is_little_endian)
			return false; // need to implement endianness conversion std::for_each(begin, end, htons) or better use transform to avoid re-reading
		if(n_length == size_t(-1))
			n_length = wcslen(p_data);
		if(!stl_ut::Reserve_N(r_s_string, n_length))
			return false;
		r_s_string.insert(r_s_string.begin(), p_data, p_data + n_length);
		return true;
	case 1:
		return n_UTF8_to_UTF16(p_data, n_length, r_s_string, b_allow_wchar_bom,
			b_use_utf16_bom, b_utf16_little_endian) != -1;
	default:
		return false; // not sure what wchar_t is
	}
}

bool CUniConv::Wide_to_UTF32(const wchar_t *p_data, size_t n_length, wstring32 &r_s_string,
	bool b_use_utf32_bom, bool b_allow_wchar_bom /*= false*/, bool b_utf32_little_endian /*= true*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		if(n_length == size_t(-1))
			n_length = wcslen(p_data);
		if(!stl_ut::Reserve_N(r_s_string, n_length))
			return false;
		r_s_string.insert(r_s_string.begin(), p_data, p_data + n_length);
		return true;
	case 2:
		return n_UTF16_to_UTF32(p_data, n_length * 2, r_s_string, b_allow_wchar_bom,
			b_is_little_endian, b_use_utf32_bom, b_utf32_little_endian) != -1;
	case 1:
		return n_UTF8_to_UTF32(p_data, n_length, r_s_string, b_allow_wchar_bom,
			b_use_utf32_bom, b_utf32_little_endian) != -1;
	default:
		return false; // not sure what wchar_t is
	}
}

bool CUniConv::UTF8_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
	bool b_use_wchar_bom, bool b_allow_utf8_bom /*= false*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		return n_UTF8_to_UTF32(p_data, n_size, *(wstring32*)&r_s_string, b_allow_utf8_bom,
			b_use_wchar_bom, b_is_little_endian) != -1;
	case 2:
		return n_UTF8_to_UTF16(p_data, n_size, *(wstring16*)&r_s_string, b_allow_utf8_bom,
			b_use_wchar_bom, b_is_little_endian) != -1;
	case 1:
		if(n_size != size_t(-1)) {
			if(!stl_ut::Reserve_N(r_s_string, n_size))
				return false;
			r_s_string.insert(r_s_string.begin(), (const wchar_t*)p_data, (const wchar_t*)p_data + n_size);
			return true;
		}
		return stl_ut::AssignWCStr(r_s_string, (const wchar_t*)p_data);
	default:
		return false; // not sure what wchar_t is
	}
}

bool CUniConv::UTF16_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
	bool b_use_wchar_bom, bool b_allow_utf16_bom /*= false*/, bool b_expect_utf16_little_endian /*= true*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		return n_UTF16_to_UTF32(p_data, n_size, *(wstring32*)&r_s_string, b_allow_utf16_bom,
			b_expect_utf16_little_endian, b_use_wchar_bom, b_is_little_endian) != -1;
	case 2:
		if(b_expect_utf16_little_endian != b_is_little_endian)
			return false; // need to implement endianness conversion std::for_each(begin, end, htons) or better use transform to avoid re-reading
		if(n_size != size_t(-1)) {
			if(!stl_ut::Reserve_N(r_s_string, n_size / 2))
				return false;
			r_s_string.insert(r_s_string.begin(), (const wchar_t*)p_data, (const wchar_t*)p_data + n_size / 2);
			return true;
		}
		return stl_ut::AssignWCStr(r_s_string, (const wchar_t*)p_data);
	case 1:
		return n_UTF16_to_UTF8(p_data, n_size, *(std::string*)&r_s_string, b_use_wchar_bom,
			b_allow_utf16_bom, b_expect_utf16_little_endian) != -1;
	default:
		return false; // not sure what wchar_t is
	}
}

bool CUniConv::UTF32_to_Wide(const void *p_data, size_t n_size, std::basic_string<wchar_t> &r_s_string,
	bool b_use_wchar_bom, bool b_allow_utf32_bom /*= false*/, bool b_expect_utf32_little_endian /*= true*/)
{
	const int n_one = 1;
	const bool b_is_little_endian = (1 == *(unsigned char*)&n_one);
	// neet to detect how the endianness will be (runitme is fine here)

	switch(sizeof(wchar_t)) {
	case 4:
		if(b_expect_utf32_little_endian != b_is_little_endian)
			return false; // need to implement endianness conversion std::for_each(begin, end, htons) or better use transform to avoid re-reading
		if(n_size != size_t(-1)) {
			if(!stl_ut::Reserve_N(r_s_string, n_size / 4))
				return false;
			r_s_string.insert(r_s_string.begin(), (const wchar_t*)p_data, (const wchar_t*)p_data + n_size / 4);
			return true;
		}
		return stl_ut::AssignWCStr(r_s_string, (const wchar_t*)p_data);
	case 2:
		return UTF32_to_UTF16((const int*)p_data, n_size / 4, b_allow_utf32_bom, b_expect_utf32_little_endian,
			*(wstring16*)&r_s_string, b_use_wchar_bom, b_is_little_endian);
	case 1:
		return UTF32_to_UTF8((const int*)p_data, n_size / 4, b_allow_utf32_bom, b_expect_utf32_little_endian,
			*(std::string*)&r_s_string, b_use_wchar_bom);
	default:
		return false; // not sure what wchar_t is
	}
}

//		--- UTF-32 functions ---

bool CUniConv::Decode_UTF32(const int *p_data, size_t n_length,
	std::string &r_s_string, bool b_allow_bom, bool b_expect_little_endian)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string

	if(n_length && b_allow_bom) {
		if(*p_data == *(const int*)p_UTF32_LE_ByteOrderMark()) {
			_ASSERTE(*p_data == 0xfffe0000);
			b_expect_little_endian = true;
			++ p_data;
			-- n_length;
		} else if(*p_data == *(const int*)p_UTF32_BE_ByteOrderMark()) {
			_ASSERTE(*p_data == 0x0000feff);
			b_expect_little_endian = false;
			++ p_data;
			-- n_length;
		}
	}
	// read byte order mark, if present

	if(b_expect_little_endian) {
		for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
			int n_code = *p_data;
			// read character

			if(!n_code)
				break;
			// may as well be null-terminated

			if(n_code > 0xff)
				n_code = '?';
			// don't know how to handle otherwise. and don't need to.

			r_s_string += char(n_code);
		}
	} else {
		for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
			int n_code = *p_data;
			// read character

			if(!n_code)
				break;
			// may as well be null-terminated

			n_code = n_ByteSwap32(n_code);
			// byte swap

			if(n_code > 0xff)
				n_code = '?';
			// don't know how to handle otherwise. and don't need to.

			r_s_string += char(n_code);
		}
	}

	return true;
}

bool CUniConv::Decode_UTF32(const int *p_data, size_t n_length,
	std::string &r_s_string, const CUnicodeMapping &r_map,
	bool b_allow_bom, bool b_expect_little_endian)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string

	if(n_length && b_allow_bom) {
		if(*p_data == *(const int*)p_UTF32_LE_ByteOrderMark()) {
			_ASSERTE(*p_data == 0xfffe0000);
			b_expect_little_endian = true;
			++ p_data;
			-- n_length;
		} else if(*p_data == *(const int*)p_UTF32_BE_ByteOrderMark()) {
			_ASSERTE(*p_data == 0x0000feff);
			b_expect_little_endian = false;
			++ p_data;
			-- n_length;
		}
	}
	// read byte order mark, if present

	if(b_expect_little_endian) {
		for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
			int n_code = *p_data;
			// read character

			if(!n_code)
				break;
			// may as well be null-terminated

			if((n_code = r_map.n_FromUnicode(n_code)) < 0)
				return false;
			// translate to 8-bit charset

			r_s_string += char(n_code);
		}
	} else {
		for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
			int n_code = *p_data;
			// read character

			if(!n_code)
				break;
			// may as well be null-terminated

			n_code = n_ByteSwap32(n_code);
			// byte swap

			if((n_code = r_map.n_FromUnicode(n_code)) < 0)
				return false;
			// translate to 8-bit charset

			r_s_string += char(n_code);
		}
	}

	return true;
}

bool CUniConv::Encode_UTF32(const char *p_data, size_t n_length,
	const int *p_mapping_table, wstring32 &r_s_string,
	bool b_use_bom, bool b_little_endian)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((b_use_bom)? 1 : 0) +
	   ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string (may not be enough this is lower-bound)

	if(b_use_bom) {
		if(b_little_endian) {
			r_s_string += char32(0xfffe0000);
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_LE_ByteOrderMark());
		} else {
			r_s_string += char32(0x0000feff);
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_BE_ByteOrderMark());
		}
	}
	// begin with bom

	for(const char *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
		int n_code = int((unsigned char)(*p_data));
		if(!n_code) // may as well be null-terminated
			break;
		// read character

		n_code = p_mapping_table[n_code];
		if(n_code < 0)
			return false;
		// translate to UTF-32 using the table

		r_s_string += char32((b_little_endian)? n_code : n_ByteSwap32(n_code));
		// write to output string
	}

	return true;
}

bool CUniConv::UTF32_to_UTF8(const int *p_data, size_t n_length,
	bool b_allow_utf32_bom, bool b_expect_utf32_little_endian,
	std::string &r_s_string, bool b_use_utf8_bom)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((b_use_utf8_bom)? 3 : 0) +
	   ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string (may not be enough,
	// this is lower-bound; should be enough for us-english though)

	if(n_length && b_allow_utf32_bom) {
		if(*p_data == *(const int*)p_UTF32_LE_ByteOrderMark()) {
			b_expect_utf32_little_endian = true;
			++ p_data;
			-- n_length;
		} else if(*p_data == *(const int*)p_UTF32_BE_ByteOrderMark()) {
			b_expect_utf32_little_endian = false;
			++ p_data;
			-- n_length;
		}
	}
	// read byte order mark, if present

	if(b_use_utf8_bom) {
		r_s_string += char(0xef);
		r_s_string += char(0xbb);
		r_s_string += char(0xbf);
	}
	// begin with UTF-8 BOM

	for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
		int n_code = *p_data;
		// read character

		if(!n_code)
			break;
		// may as well be null-terminated

		if(!b_expect_utf32_little_endian)
			n_code = n_ByteSwap32(n_code);
		// byte swap

		if(n_code <= 0x7f) {
			if(!stl_ut::Reserve_1More(r_s_string))
				return false;
			// make sure there's enough space

			r_s_string += char(n_code);
			// save as a single value
		} else if(n_code <= 0x7ff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return false;
			// make sure there's enough space

			r_s_string += char(0xc0 | (n_code >> 6));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as pair of values
		} else if(n_code <= 0xffff) {
			if(n_code >= 0xd800 && n_code <= 0xdfff)
				return false;
			// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

			if(!stl_ut::Reserve_NMore(r_s_string, 3))
				return false;
			// make sure there's enough space

			r_s_string += char(0xe0 | (n_code >> 12));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as trinity of values
		} else if(n_code <= 0x10ffff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 4))
				return false;
			// make sure there's enough space

			r_s_string += char(0xf0 | (n_code >> 18));
			r_s_string += char(0x80 | ((n_code >> 12) & 0x3f));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as quadruple of values
		} else {
			return false;
			// too high character to encode
		}
	}

	return true;
}

bool CUniConv::UTF32_to_UTF16(const int *p_data, size_t n_length,
	bool b_allow_utf32_bom, bool b_expect_utf32_little_endian,
	wstring16 &r_s_string, bool b_use_utf16_bom, bool b_utf16_little_endian)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((b_use_utf16_bom)? 3 : 0) +
	   ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string (may not be enough this is lower-bound)

	if(n_length && b_allow_utf32_bom) {
		if(*p_data == *(const int*)p_UTF32_LE_ByteOrderMark()) {
			b_expect_utf32_little_endian = true;
			++ p_data;
			-- n_length;
		} else if(*p_data == *(const int*)p_UTF32_BE_ByteOrderMark()) {
			b_expect_utf32_little_endian = false;
			++ p_data;
			-- n_length;
		}
	}
	// read byte order mark, if present

	if(b_use_utf16_bom) {
		r_s_string += (b_utf16_little_endian)? 0xfeff : 0xfffe;
		_ASSERTE(r_s_string[0] = *(const char16*)((b_utf16_little_endian)?
			p_UTF16_LE_ByteOrderMark() : p_UTF16_BE_ByteOrderMark())); // make sure the byte mark is correct
	}
	// include BOM

	for(const int *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
		int n_code = *p_data;
		// read character

		if(!n_code)
			break;
		// may as well be null-terminated

		if(!b_expect_utf32_little_endian)
			n_code = n_ByteSwap32(n_code);
		// byte swap

		if(n_code < 0x10000) {
			if(n_code >= 0xd800 && n_code <= 0xdfff)
				return false;
			// noncharacters

			if(!stl_ut::Reserve_1More(r_s_string))
				return false;
			// make sure there's enough space

			r_s_string += char16((b_utf16_little_endian)? uint16_t(n_code) : n_HiLoSwap(n_code));
			// save as a single value
		} else if(n_code <= 0x10ffff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return false;
			// make sure there's enough space

			uint16_t n_head = uint16_t((0xd800 - (0x10000 >> 10)) + (n_code >> 10));
			r_s_string += char16((b_utf16_little_endian)? n_head : n_HiLoSwap(n_head));
			uint16_t n_tail = uint16_t(0xdc00 + (n_code & 0x3ff));
			r_s_string += char16((b_utf16_little_endian)? n_tail : n_HiLoSwap(n_tail));
			// save as surrogate pair
		} else {
			return false;
			// too high character to encode
		}
	}

	return true;
}

//		--- UTF-16 functions ---

int CUniConv::n_Decode_UTF16(const void *p_data, size_t n_size,
	std::string &r_s_string, bool b_allow_bom, bool b_expect_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	n_size &= ~1;
	// size must be even

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size / 2))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	bool b_little_endian = b_expect_little_endian;
	// use default endianness

	if(n_size > 2 && b_allow_bom) {
		if(*(const uint16_t*)p_data8 == *(const uint16_t*)p_UTF16_LE_ByteOrderMark()) {
			_ASSERTE(p_data8[0] == 0xff && p_data8[1] == 0xfe);
			b_little_endian = true;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		} else if(*(const uint16_t*)p_data8 == *(const uint16_t*)p_UTF16_BE_ByteOrderMark()) {
			_ASSERTE(p_data8[0] == 0xfe && p_data8[1] == 0xff);
			b_little_endian = false;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		}
		// skip BOM
	}
	// try to read BOM (if present and allowed)

	const int n_i0 = (b_little_endian)? 1 : 0;
	const int n_i1 = (b_little_endian)? 0 : 1;
	// byte indexing (endianness)

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   p_char += 2, n_read += 2) {
		if(p_char[0] == 0 && p_char[1] == 0) {
			n_read += 2;
			break;
		}
		// can be null-terminated

		int n_code = (p_char[n_i0] << 8) | p_char[n_i1];
		// read code

		if((n_code >> 10) == 0x36) {
			// n_code is a high surrogate

			p_char += 2;
			n_read += 2;
			if(p_char >= p_end)
				return -1; // not enough data
			int n_code2 = (p_char[n_i0] << 8) | p_char[n_i1];
			if((n_code2 >> 10) != 0x37)
				return -1;
			// read low surrogate

			n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
			// have surrogate pair
		} else if((n_code >> 10) == 0x37)
			return -1; // lonely low surrogate
		// read surrogate pairs

		if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
			return -1;
		// check if code is valid character

		if(n_code > 0xff)
			n_code = '?';
		// don't know how to handle otherwise. and don't need to.

		r_s_string += char(n_code);
	}

	return n_read;
}

int CUniConv::n_Decode_UTF16(const void *p_data, size_t n_size, std::string &r_s_string,
	const CUnicodeMapping &r_map, bool b_allow_bom, bool b_expect_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	n_size &= ~1;
	// size must be even

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size / 2))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	bool b_little_endian = b_expect_little_endian;
	// use default endianness

	if(n_size > 2 && b_allow_bom) {
		if(*(const uint16_t*)p_data8 == *(const uint16_t*)p_UTF16_LE_ByteOrderMark()) {
			_ASSERTE(p_data8[0] == 0xff && p_data8[1] == 0xfe);
			b_little_endian = true;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		} else if(*(const uint16_t*)p_data8 == *(const uint16_t*)p_UTF16_BE_ByteOrderMark()) {
			_ASSERTE(p_data8[0] == 0xfe && p_data8[1] == 0xff);
			b_little_endian = false;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		}
		// skip BOM
	}
	// try to read BOM (if present and allowed)

	const int n_i0 = (b_little_endian)? 1 : 0;
	const int n_i1 = (b_little_endian)? 0 : 1;
	// byte indexing (endianness)

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   p_char += 2, n_read += 2) {
		if(p_char[0] == 0 && p_char[1] == 0) {
			n_read += 2;
			break;
		}
		// can be null-terminated

		int n_code = (p_char[n_i0] << 8) | p_char[n_i1];
		// read code

		if((n_code >> 10) == 0x36) {
			// n_code is a high surrogate

			p_char += 2;
			n_read += 2;
			if(p_char >= p_end)
				return -1; // not enough data
			int n_code2 = (p_char[n_i0] << 8) | p_char[n_i1];
			if((n_code2 >> 10) != 0x37)
				return -1;
			// read low surrogate

			n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
			// have surrogate pair
		} else if((n_code >> 10) == 0x37)
			return -1; // lonely low surrogate
		// read surrogate pairs

		if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
			return -1;
		// check if code is valid character

		if((n_code = r_map.n_FromUnicode(n_code)) < 0)
			return -1;
		// translate to 8-bit charset

		r_s_string += char(n_code);
	}

	return n_read;
}

bool CUniConv::Encode_UTF16(const char *p_data, size_t n_length,
	const int *p_mapping_table, wstring16 &r_s_string,
	bool b_use_bom, bool b_little_endian)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((b_use_bom)? 1 : 0) +
	   ((n_length == size_t(-1))? 0 : n_length)))
		return false;
	// reserve some space in the string (may not be enough this is lower-bound)

	if(b_use_bom)
		r_s_string += (b_little_endian)? 0xfeff : 0xfffe;
	// include BOM

	for(const char *p_end = p_data + n_length; p_data != p_end; ++ p_data) {
		int n_code = int((unsigned char)(*p_data));
		if(!n_code) // may as well be null-terminated
			break;
		// read character

		n_code = p_mapping_table[n_code];
		if(n_code < 0)
			return false;
		// translate to UTF-32 using the table

		if(n_code < 0x10000) {
			if(n_code >= 0xd800 && n_code <= 0xdfff)
				return false;
			// noncharacters

			if(!stl_ut::Reserve_1More(r_s_string))
				return false;
			// make sure there's enough space

			r_s_string += char16((b_little_endian)? uint16_t(n_code) : n_HiLoSwap(n_code));
			// save as a single value
		} else if(n_code <= 0x10ffff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return false;
			// make sure there's enough space

			uint16_t n_head = uint16_t((0xd800 - (0x10000 >> 10)) + (n_code >> 10));
			r_s_string += char16((b_little_endian)? n_head : n_HiLoSwap(n_head));
			uint16_t n_tail = uint16_t(0xdc00 + (n_code & 0x3ff));
			r_s_string += char16((b_little_endian)? n_tail : n_HiLoSwap(n_tail));
			// save as surrogate pair
		} else {
			return false;
			// too high character to encode
		}
	}

	return true;
}

int CUniConv::n_UTF16_to_UTF32(const void *p_data, size_t n_size,
	wstring32 &r_s_string, bool b_allow_utf16_bom,
	bool b_expect_utf16_little_endian,
	bool b_use_utf32_bom, bool b_utf32_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	n_size &= ~1;
	// size must be even

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size / 2 + ((b_use_utf32_bom)? 1 : 0)))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	if(n_size > 2 && b_allow_utf16_bom) {
		if(p_data8[0] == 0xff && p_data8[1] == 0xfe) {
			b_expect_utf16_little_endian = true;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		} else if(p_data8[0] == 0xfe && p_data8[1] == 0xff) {
			b_expect_utf16_little_endian = false;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		}
		// skip BOM
	}
	// try to read BOM (if present and allowed)

	if(b_use_utf32_bom) {
		if(b_utf32_little_endian) {
			r_s_string += 0xfffe0000;
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_LE_ByteOrderMark());
		} else {
			r_s_string += 0x0000feff;
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_BE_ByteOrderMark());
		}
	}
	// begin with bom

	const int n_i0 = (b_expect_utf16_little_endian)? 1 : 0;
	const int n_i1 = (b_expect_utf16_little_endian)? 0 : 1;
	// byte indexing (endianness)

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   p_char += 2, n_read += 2) {
		if(p_char[0] == 0 && p_char[1] == 0) {
			n_read += 2;
			break;
		}
		// can be null-terminated

		int n_code = (p_char[n_i0] << 8) | p_char[n_i1];
		// read code

		if((n_code >> 10) == 0x36) {
			// n_code is a high surrogate

			p_char += 2;
			n_read += 2;
			if(p_char >= p_end)
				return -1; // not enough data
			int n_code2 = (p_char[n_i0] << 8) | p_char[n_i1];
			if((n_code2 >> 10) != 0x37)
				return -1;
			// read low surrogate

			n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
			// have surrogate pair
		} else if((n_code >> 10) == 0x37)
			return -1; // lonely low surrogate
		// read surrogate pairs

		if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
			return -1;
		// check if code is valid character

		r_s_string += char32((b_utf32_little_endian)? n_code : n_ByteSwap32(n_code));
	}

	return n_read;
}

int CUniConv::n_UTF16_to_UTF8(const void *p_data, size_t n_size,
	std::string &r_s_string, bool b_use_utf8_bom, bool b_allow_utf16_bom,
	bool b_expect_utf16_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	n_size &= ~1;
	// size must be even

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size / 2 + ((b_use_utf8_bom)? 3 : 0)))
		return -1;
	// reserve enough space in the string

	if(b_use_utf8_bom) {
		r_s_string += char(0xef);
		r_s_string += char(0xbb);
		r_s_string += char(0xbf);
	}
	// begin with utf-8 BOM

	int n_read = 0;
	// number of bytes read

	bool b_little_endian = b_expect_utf16_little_endian;
	// use default endianness

	if(n_size > 2 && b_allow_utf16_bom) {
		if(p_data8[0] == 0xff && p_data8[1] == 0xfe) {
			b_little_endian = true;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		} else if(p_data8[0] == 0xfe && p_data8[1] == 0xff) {
			b_little_endian = false;
			n_size -= 2;
			p_data8 += 2;
			n_read += 2;
		}
		// skip BOM
	}
	// try to read BOM (if present and allowed)

	const int n_i0 = (b_little_endian)? 1 : 0;
	const int n_i1 = (b_little_endian)? 0 : 1;
	// byte indexing (endianness)

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   p_char += 2, n_read += 2) {
		if(p_char[0] == 0 && p_char[1] == 0) {
			n_read += 2;
			break;
		}
		// can be null-terminated

		int n_code = (p_char[n_i0] << 8) | p_char[n_i1];
		// read code

		if((n_code >> 10) == 0x36) { // high surrogate
			p_char += 2;
			n_read += 2;
			if(p_char >= p_end)
				return -1; // not enough data
			int n_code2 = (p_char[n_i0] << 8) | p_char[n_i1];
			if((n_code2 >> 10) != 0x37)
				return -1;
			// read low surrogate

			n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
			// have surrogate pair
		} else if((n_code >> 10) == 0x37)
			return -1; // lonely low surrogate
		// read surrogate pairs

		if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
			return -1;
		// check if code is valid character

		if(n_code <= 0x7f) {
			if(!stl_ut::Reserve_1More(r_s_string))
				return -1;
			// make sure there's enough space

			r_s_string += char(n_code);
			// save as a single value
		} else if(n_code <= 0x7ff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return -1;
			// make sure there's enough space

			r_s_string += char(0xc0 | (n_code >> 6));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as pair of values
		} else if(n_code <= 0xffff) {
			/*if(n_code >= 0xd800 && n_code <= 0xdfff) // already checked above
				return false;*/
			// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

			if(!stl_ut::Reserve_NMore(r_s_string, 3))
				return -1;
			// make sure there's enough space

			r_s_string += char(0xe0 | (n_code >> 12));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as trinity of values
		} else /*if(n_code <= 0x10ffff)*/ {
			if(!stl_ut::Reserve_NMore(r_s_string, 4))
				return -1;
			// make sure there's enough space

			r_s_string += char(0xf0 | (n_code >> 18));
			r_s_string += char(0x80 | ((n_code >> 12) & 0x3f));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as quadruple of values
		} /*else {
			return false;
			// too high character to encode // already checked above
		}*/
		// convert to UTF-8
	}

	return n_read;
}

int CUniConv::n_UTF16_Char_Size(uint16_t n_first_word)
{
	int n_code = n_first_word;
	if((n_code >> 10) == 0x36)
		return 4;
	else if((n_code >> 10) == 0x37) // lonely low surrogate
		return -1;
	return 2;
}

int CUniConv::n_UTF16_LE_Char_Size(uint8_t UNUSED(n_first_byte), uint8_t n_second_byte)
{
	/*int n_code = (n_second_byte << 8) | n_first_byte;
	if((n_code >> 10) == 0x36)
		return 4;*/
	_ASSERTE((((n_second_byte << 8) | n_first_byte) >> 10) == (n_second_byte << 2));
	if((n_second_byte << 2) == 0x36) // can decide based on a single byte
		return 4;
	else if((n_second_byte << 2) == 0x37) // lonely low surrogate
		return -1;
	return 2;
}

int CUniConv::n_UTF16_BE_Char_Size(uint8_t n_first_byte, uint8_t n_second_byte)
{
	/*int n_code = (n_first_byte << 8) | n_second_byte;
	if((n_code >> 10) == 0x36)
		return 4;*/
	_ASSERTE((((n_first_byte << 8) | n_second_byte) >> 10) == (n_first_byte << 2));
	if((n_first_byte << 2) == 0x36) // can decide based on a single byte
		return 4;
	else if((n_second_byte << 2) == 0x37) // lonely low surrogate
		return -1;
	return 2;
}

int CUniConv::n_UTF16_LE_Code(const void *p_data, size_t n_size, int &r_n_read)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	if(n_size < 2) {
		r_n_read = 0;
		return -1;
	}
	// make sure there are at least two characters

	int n_code = (p_data8[1] << 8) | p_data8[0];
	// read code

	if((n_code >> 10) == 0x36) {
		// n_code is a high surrogate

		p_data8 += 2;
		if(n_size < 4) {
			r_n_read = 2;
			return -1; // not enough data
		}
		int n_code2 = (p_data8[1] << 8) | p_data8[0];
		if((n_code2 >> 10) != 0x37) {
			r_n_read = 2;
			return -1;
		}
		// read low surrogate

		r_n_read = 4;
		n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
		// have surrogate pair
	} else {
		r_n_read = 2;
		if((n_code >> 10) == 0x37) // lonely low surrogate
			return -1;
		// single character
	}
	// read surrogate pairs

	if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
		return -1;
	// check if code is valid character

	return n_code;
}

int CUniConv::n_UTF16_BE_Code(const void *p_data, size_t n_size, int &r_n_read)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	if(n_size < 2) {
		r_n_read = 0;
		return -1;
	}
	// make sure there are at least two characters

	int n_code = (p_data8[0] << 8) | p_data8[1];
	// read code

	if((n_code >> 10) == 0x36) {
		// n_code is a high surrogate

		p_data8 += 2;
		if(n_size < 4) {
			r_n_read = 2;
			return -1; // not enough data
		}
		int n_code2 = (p_data8[0] << 8) | p_data8[1];
		if((n_code2 >> 10) != 0x37) {
			r_n_read = 2;
			return -1;
		}
		// read low surrogate

		r_n_read = 4;
		n_code = (((n_code & 0x3ff) << 10) | (n_code2 & 0x3ff)) + 0x10000;
		// have surrogate pair
	} else {
		r_n_read = 2;
		if((n_code >> 10) == 0x37) // lonely low surrogate
			return -1;
		// single character
	}
	// read surrogate pairs

	if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
		return -1;
	// check if code is valid character

	return n_code;
}

int CUniConv::n_UTF16_Encode_Size(int n_char)
{
	_ASSERTE(n_char <= 0x10ffff);
	if(n_char <= 0x10000)
		return 2;
	else //if(n_char <= 0x10ffff)
		return 4;
}

int CUniConv::n_UTF16_SafeEncode_Size(int n_char)
{
	if(n_char <= 0x10000)
		return 2;
	else if(n_char <= 0x10ffff)
		return 4;
	return -1;
}

int CUniConv::n_EncodeCharacter_UTF16_LE(void *p_dest, size_t n_space, int n_character)
{
	uint16_t *p_dest16 = (uint16_t*)p_dest;

	if(n_character < 0x10000) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return 0;
		// noncharacters

		if(n_space < 2)
			return 0;
		// make sure there's enough space

		p_dest16[0] = uint16_t(n_character);
		// save as a single value

		return 2;
	} else if(n_character <= 0x10ffff) {
		if(n_space < 4)
			return 0;
		// make sure there's enough space

		p_dest16[0] = uint16_t((0xd800 - (0x10000 >> 10)) + (n_character >> 10));
		p_dest16[1] = uint16_t(0xdc00 + (n_character & 0x3ff));
		// save as surrogate pair

		return 4;
	}

	return 0;
	// too high character to encode
}

int CUniConv::n_EncodeCharacter_UTF16_BE(void *p_dest, size_t n_space, int n_character)
{
	uint16_t *p_dest16 = (uint16_t*)p_dest;

	if(n_character < 0x10000) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return 0;
		// noncharacters

		if(n_space < 2)
			return 0;
		// make sure there's enough space

		p_dest16[0] = n_HiLoSwap(n_character);
		// save as a single value

		return 2;
	} else if(n_character <= 0x10ffff) {
		if(n_space < 4)
			return 0;
		// make sure there's enough space

		p_dest16[0] = n_HiLoSwap((0xd800 - (0x10000 >> 10)) + (n_character >> 10));
		p_dest16[1] = n_HiLoSwap(0xdc00 + (n_character & 0x3ff));
		// save as surrogate pair

		return 4;
	}

	return 0;
	// too high character to encode
}

bool CUniConv::EncodeCharacter_UTF16_LE_Strict(void *p_dest, size_t n_space, int n_character)
{
	uint16_t *p_dest16 = (uint16_t*)p_dest;

	if(n_character < 0x10000) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return false;
		// noncharacters

		if(n_space != 2)
			return false;
		// make sure there's enough space

		p_dest16[0] = uint16_t(n_character);
		// save as a single value

		return true;
	} else if(n_character <= 0x10ffff) {
		if(n_space != 4)
			return false;
		// make sure there's enough space

		p_dest16[0] = uint16_t((0xd800 - (0x10000 >> 10)) + (n_character >> 10));
		p_dest16[1] = uint16_t(0xdc00 + (n_character & 0x3ff));
		// save as surrogate pair

		return true;
	}

	return false;
	// too high character to encode
}

bool CUniConv::EncodeCharacter_UTF16_BE_Strict(void *p_dest, size_t n_space, int n_character)
{
	uint16_t *p_dest16 = (uint16_t*)p_dest;

	if(n_character < 0x10000) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return false;
		// noncharacters

		if(n_space != 2)
			return false;
		// make sure there's enough space

		p_dest16[0] = n_HiLoSwap(n_character);
		// save as a single value

		return true;
	} else if(n_character <= 0x10ffff) {
		if(n_space != 4)
			return false;
		// make sure there's enough space

		p_dest16[0] = n_HiLoSwap((0xd800 - (0x10000 >> 10)) + (n_character >> 10));
		p_dest16[1] = n_HiLoSwap(0xdc00 + (n_character & 0x3ff));
		// save as surrogate pair

		return true;
	}

	return false;
	// too high character to encode
}

//		--- UTF-8 functions ---

int CUniConv::n_Decode_UTF8(const void *p_data,
	size_t n_size, std::string &r_s_string, bool b_allow_bom)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	if(b_allow_bom && n_size >= 3 && p_data8[0] == 0xef &&
	   p_data8[1] == 0xbb && p_data8[2] == 0xbf) {
		p_data8 += 3;
		n_size -= 3;
		n_read = 3;
	}
	// skip UTF-8 BOM

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   ++ p_char, ++ n_read) {
		if(!*p_char) {
			++ n_read;
			break;
		}
		// can be null-terminated

		uint8_t n_byte0 = *p_char;
		// get byte

		int n_code;
		if((n_byte0 & 0x80) != 0) {
			int n_byte_num;
			if((n_byte0 & 0xe0) == 0xc0)
				n_byte_num = 1;
			else if((n_byte0 & 0xf0) == 0xe0)
				n_byte_num = 2;
			else if((n_byte0 & 0xf8) == 0xf0)
				n_byte_num = 3;
			else
				return -1;
			n_code = n_byte0 & (0x3f >> n_byte_num);
			// multi-byte character - apply mask and determine number of bytes

			if(p_char + n_byte_num >= p_end)
				return -1;
			// have enough data?

			for(; n_byte_num; -- n_byte_num) {
				++ p_char;
				++ n_read;
				uint8_t n_byte = *p_char;
				if((n_byte & 0xc0) != 0x80)
					return -1;
				n_code <<= 6;
				n_code |= n_byte & 0x3f;
			}
			// add more bytes

			if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
				return -1;
			// check if code is valid character (UTF-16 surrogates not allowed)
		} else
			n_code = n_byte0; // single-byte
		// decode utf-8 char

		if(n_code > 0xff)
			n_code = '?';
		// don't know how to handle otherwise. and don't need to.

		r_s_string += char(n_code);
	}

	return n_read;
}

int CUniConv::n_Decode_UTF8(const void *p_data, size_t n_size,
	std::string &r_s_string, const CUnicodeMapping &r_map, bool b_allow_bom)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	if(b_allow_bom && n_size >= 3 && p_data8[0] == 0xef &&
	   p_data8[1] == 0xbb && p_data8[2] == 0xbf) {
		p_data8 += 3;
		n_size -= 3;
		n_read = 3;
	}
	// skip UTF-8 BOM

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   ++ p_char, ++ n_read) {
		if(!*p_char) {
			++ n_read;
			break;
		}
		// can be null-terminated

		uint8_t n_byte0 = *p_char;
		// get byte

		int n_code;
		if((n_byte0 & 0x80) != 0) {
			int n_byte_num;
			if((n_byte0 & 0xe0) == 0xc0)
				n_byte_num = 1;
			else if((n_byte0 & 0xf0) == 0xe0)
				n_byte_num = 2;
			else if((n_byte0 & 0xf8) == 0xf0)
				n_byte_num = 3;
			else
				return -1;
			n_code = n_byte0 & (0x3f >> n_byte_num);
			// multi-byte character - apply mask and determine number of bytes

			if(p_char + n_byte_num >= p_end)
				return -1;
			// have enough data?

			for(; n_byte_num; -- n_byte_num) {
				++ p_char;
				++ n_read;
				uint8_t n_byte = *p_char;
				if((n_byte & 0xc0) != 0x80)
					return -1;
				n_code <<= 6;
				n_code |= n_byte & 0x3f;
			}
			// add more bytes

			if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
				return -1;
			// check if code is valid character (UTF-16 surrogates not allowed)
		} else
			n_code = n_byte0; // single-byte
		// decode utf-8 char

		if((n_code = r_map.n_FromUnicode(n_code)) < 0)
			return -1;
		// translate to 8-bit charset

		r_s_string += char(n_code);
	}

	return n_read;
}

int CUniConv::n_UTF8_Char_Size(uint8_t n_first_byte)
{
	if(n_first_byte & 0x80) {
		if((n_first_byte & 0xe0) == 0xc0)
			return 2;
		else if((n_first_byte & 0xf0) == 0xe0)
			return 3;
		else if((n_first_byte & 0xf8) == 0xf0)
			return 4;
		else
			return -1;
	} else
		return 1;
}

int CUniConv::n_UTF8_Code(const void *p_data, size_t n_size, int &r_n_read)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	if(!n_size) {
		r_n_read = 0;
		return -1;
	}
	// make sure there's at least a single characer

	uint8_t n_byte0 = *p_data8;
	// get first byte

	if((n_byte0 & 0x80) != 0) {
		unsigned int n_byte_num;
		if((n_byte0 & 0xe0) == 0xc0)
			n_byte_num = 1;
		else if((n_byte0 & 0xf0) == 0xe0)
			n_byte_num = 2;
		else if((n_byte0 & 0xf8) == 0xf0)
			n_byte_num = 3;
		else {
			r_n_read = 1;
			return -1;
		}
		// multi-byte character: determine number of bytes

		if(n_byte_num >= n_size) {
			r_n_read = 1;
			return -1;
		}
		// have enough data?

		int n_code = n_byte0 & (0x3f >> n_byte_num);
		r_n_read = 1 + n_byte_num;
		for(; n_byte_num; -- n_byte_num) {
			uint8_t n_byte = *(++ p_data8);
			if((n_byte & 0xc0) != 0x80) {
				r_n_read -= n_byte_num - 1; // correct number of bytes actually read
				return -1;
			}
			n_code = (n_code << 6) | (n_byte & 0x3f);
		}
		// add more bytes

		if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
			return -1;
		// check if code is valid character (UTF-16 surrogates not allowed)

		return n_code;
	} else {
		r_n_read = 1;
		return n_byte0; // single-byte
	}
	// decode utf-8 char
}

bool CUniConv::Encode_UTF8(const char *p_data, size_t n_size,
	const int *p_mapping_table, std::string &r_s_string, bool b_use_bom)
{
	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, ((b_use_bom)? 3 : 0) +
	   ((n_size == size_t(-1))? 0 : n_size)))
		return false;
	// reserve some space in the string (may not be enough,
	// this is lower-bound; should be enough for us-english though)

	if(b_use_bom) {
		r_s_string += char(0xef);
		r_s_string += char(0xbb);
		r_s_string += char(0xbf);
	}
	// begin with UTF-8 BOM

	for(const char *p_end = p_data + n_size; p_data != p_end; ++ p_data) {
		int n_code = int((unsigned char)(*p_data));
		if(!n_code) // may as well be null-terminated
			break;
		// read character

		n_code = p_mapping_table[n_code];
		if(n_code < 0)
			return false;
		// translate to UTF-32 using the table

		if(n_code <= 0x7f) {
			if(!stl_ut::Reserve_1More(r_s_string))
				return false;
			// make sure there's enough space

			r_s_string += char(n_code);
			// save as a single value
		} else if(n_code <= 0x7ff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return false;
			// make sure there's enough space

			r_s_string += char(0xc0 | (n_code >> 6));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as pair of values
		} else if(n_code <= 0xffff) {
			if(n_code >= 0xd800 && n_code <= 0xdfff)
				return false;
			// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

			if(!stl_ut::Reserve_NMore(r_s_string, 3))
				return false;
			// make sure there's enough space

			r_s_string += char(0xe0 | (n_code >> 12));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as trinity of values
		} else if(n_code <= 0x10ffff) {
			if(!stl_ut::Reserve_NMore(r_s_string, 4))
				return false;
			// make sure there's enough space

			r_s_string += char(0xf0 | (n_code >> 18));
			r_s_string += char(0x80 | ((n_code >> 12) & 0x3f));
			r_s_string += char(0x80 | ((n_code >> 6) & 0x3f));
			r_s_string += char(0x80 | (n_code & 0x3f));
			// save as quadruple of values
		} else {
			return false;
			// too high character to encode
		}
	}

	return true;
}

int CUniConv::n_UTF8_to_UTF16(const void *p_data, size_t n_size,
	wstring16 &r_s_string, bool b_allow_utf8_bom,
	bool b_include_utf16_bom, bool b_utf16_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, (b_include_utf16_bom)? n_size + 1 : n_size))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	if(b_allow_utf8_bom && n_size >= 3 && p_data8[0] == 0xef &&
	   p_data8[1] == 0xbb && p_data8[2] == 0xbf) {
		p_data8 += 3;
		n_size -= 3;
		n_read = 3;
	}
	// skip UTF-8 BOM

	if(b_include_utf16_bom)
		r_s_string += (b_utf16_little_endian)? 0xfeff : 0xfffe;
	// include BOM

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end; ++ p_char, ++ n_read) {
		if(!*p_char) {
			++ n_read;
			break;
		}
		// can be null-terminated

		uint8_t n_byte0 = *p_char;
		// get byte

		int n_code;
		if((n_byte0 & 0x80) != 0) {
			int n_byte_num;
			if((n_byte0 & 0xe0) == 0xc0)
				n_byte_num = 1;
			else if((n_byte0 & 0xf0) == 0xe0)
				n_byte_num = 2;
			else if((n_byte0 & 0xf8) == 0xf0)
				n_byte_num = 3;
			else
				return -1;
			n_code = n_byte0 & (0x3f >> n_byte_num);
			// multi-byte character - apply mask and determine number of bytes

			if(p_char + n_byte_num >= p_end)
				return -1;
			// have enough data?

			for(; n_byte_num; -- n_byte_num) {
				++ p_char;
				++ n_read;
				uint8_t n_byte = *p_char;
				if((n_byte & 0xc0) != 0x80)
					return -1;
				n_code <<= 6;
				n_code |= n_byte & 0x3f;
			}
			// add more bytes

			if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
				return -1;
			// check if code is valid character (UTF-16 surrogates not allowed)
		} else
			n_code = n_byte0; // single-byte
		// decode utf-8 char

		if(n_code < 0x10000) {
			/*if(n_code >= 0xd800 && n_code <= 0xdfff) // already checked above
				return false;*/
			// noncharacters

			if(!stl_ut::Reserve_1More(r_s_string))
				return -1;
			// make sure there's enough space

			r_s_string += char16((b_utf16_little_endian)? uint16_t(n_code) : n_HiLoSwap(n_code));
			// save as a single value
		} else /*if(n_code <= 0x10ffff)*/ {
			if(!stl_ut::Reserve_NMore(r_s_string, 2))
				return -1;
			// make sure there's enough space

			uint16_t n_head = uint16_t((0xd800 - (0x10000 >> 10)) + (n_code >> 10));
			r_s_string += char16((b_utf16_little_endian)? n_head : n_HiLoSwap(n_head));
			uint16_t n_tail = uint16_t(0xdc00 + (n_code & 0x3ff));
			r_s_string += char16((b_utf16_little_endian)? n_tail : n_HiLoSwap(n_tail));
			// save as surrogate pair
		} /*else {
			return false;
			// too high character to encode // already checked above
		}*/
		// encode utf-16
	}

	return n_read;
}

int CUniConv::n_UTF8_to_UTF32(const void *p_data,
	size_t n_size, wstring32 &r_s_string, bool b_allow_utf8_bom,
	bool b_use_utf32_bom, bool b_utf32_little_endian)
{
	const uint8_t *p_data8 = (const uint8_t*)p_data;

	r_s_string.erase();
	if(!stl_ut::Reserve_N(r_s_string, n_size))
		return -1;
	// reserve enough space in the string

	int n_read = 0;
	// number of bytes read

	if(b_allow_utf8_bom && n_size >= 3 && p_data8[0] == 0xef &&
	   p_data8[1] == 0xbb && p_data8[2] == 0xbf) {
		p_data8 += 3;
		n_size -= 3;
		n_read = 3;
	}
	// skip UTF-8 BOM

	if(b_use_utf32_bom) {
		if(b_utf32_little_endian) {
			r_s_string += 0xfffe0000;
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_LE_ByteOrderMark());
		} else {
			r_s_string += 0x0000feff;
			_ASSERTE(r_s_string[0] == *(const int*)p_UTF32_BE_ByteOrderMark());
		}
	}
	// begin with bom

	for(const uint8_t *p_char = p_data8,
	   *p_end = p_data8 + n_size; p_char < p_end;
	   ++ p_char, ++ n_read) {
		if(!*p_char) {
			++ n_read;
			break;
		}
		// can be null-terminated

		uint8_t n_byte0 = *p_char;
		// get byte

		if((n_byte0 & 0x80) != 0) {
			int n_byte_num;
			if((n_byte0 & 0xe0) == 0xc0)
				n_byte_num = 1;
			else if((n_byte0 & 0xf0) == 0xe0)
				n_byte_num = 2;
			else if((n_byte0 & 0xf8) == 0xf0)
				n_byte_num = 3;
			else
				return -1;
			// multi-byte character: determine number of bytes

			if(p_char + n_byte_num >= p_end)
				return -1;
			// have enough data?

			int n_code = n_byte0 & (0x3f >> n_byte_num);
			n_read += n_byte_num;
			for(; n_byte_num; -- n_byte_num) {
				uint8_t n_byte = *(++ p_char);
				if((n_byte & 0xc0) != 0x80) {
					n_read -= n_byte_num - 1; // correct number of bytes actually read
					return -1;
				}
				n_code = (n_code << 6) | (n_byte & 0x3f);
			}
			// add more bytes

			if((n_code >= 0xd800 && n_code <= 0xdfff) || n_code > 0x10ffff)
				return -1;
			// check if code is valid character (UTF-16 surrogates not allowed)

			r_s_string += char32((b_utf32_little_endian)? n_code : n_ByteSwap32(n_code));
		} else
			r_s_string += char32((b_utf32_little_endian)? n_byte0 : n_ByteSwap32(n_byte0)); // single-byte
		// decode utf-8 char
	}

	return n_read;
}

int CUniConv::n_UTF8_Encode_Size(int n_char)
{
	_ASSERTE(n_char <= 0x10ffff);
	if(n_char <= 0x7f)
		return 1;
	else if(n_char <= 0x7ff)
		return 2;
	else if(n_char <= 0xffff)
		return 3;
	else //if(n_char <= 0x10ffff)
		return 4;
}

int CUniConv::n_UTF8_SafeEncode_Size(int n_char)
{
	if(n_char <= 0x7f)
		return 1;
	else if(n_char <= 0x7ff)
		return 2;
	else if(n_char <= 0xffff)
		return 3;
	else if(n_char <= 0x10ffff)
		return 4;
	return -1;
}

int CUniConv::n_EncodeCharacter_UTF8(char *p_dest, size_t n_space, int n_character)
{
	if(n_character <= 0x7f) {
		if(n_space < 1)
			return 0;
		// make sure there's enough space

		p_dest[0] = char(n_character);
		// save as a single value

		return 1;
	} else if(n_character <= 0x7ff) {
		if(n_space < 2)
			return 0;
		// make sure there's enough space

		p_dest[0] = char(0xc0 | (n_character >> 6));
		p_dest[1] = char(0x80 | (n_character & 0x3f));
		// save as pair of values

		return 2;
	} else if(n_character <= 0xffff) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return 0;
		// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

		if(n_space < 3)
			return 0;
		// make sure there's enough space

		p_dest[0] = char(0xe0 | (n_character >> 12));
		p_dest[1] = char(0x80 | ((n_character >> 6) & 0x3f));
		p_dest[2] = char(0x80 | (n_character & 0x3f));
		// save as trinity of values

		return 3;
	} else if(n_character <= 0x10ffff) {
		if(n_space < 4)
			return 0;
		// make sure there's enough space

		p_dest[0] = char(0xf0 | (n_character >> 18));
		p_dest[1] = char(0x80 | ((n_character >> 12) & 0x3f));
		p_dest[2] = char(0x80 | ((n_character >> 6) & 0x3f));
		p_dest[3] = char(0x80 | (n_character & 0x3f));
		// save as quadruple of values

		return 4;
	}

	return 0;
	// too high character to encode
}

bool CUniConv::EncodeCharacter_UTF8_Strict(char *p_dest, size_t n_space, int n_character)
{
	if(n_character <= 0x7f) {
		if(n_space != 1)
			return false;
		// make sure there's enough space

		p_dest[0] = char(n_character);
		// save as a single value
	} else if(n_character <= 0x7ff) {
		if(n_space != 2)
			return false;
		// make sure there's enough space

		p_dest[0] = char(0xc0 | (n_character >> 6));
		p_dest[1] = char(0x80 | (n_character & 0x3f));
		// save as pair of values
	} else if(n_character <= 0xffff) {
		if(n_character >= 0xd800 && n_character <= 0xdfff)
			return false;
		// can't encode utf-16 surrogates. it's prohibited in utf-8 specs.

		if(n_space != 3)
			return false;
		// make sure there's enough space

		p_dest[0] = char(0xe0 | (n_character >> 12));
		p_dest[1] = char(0x80 | ((n_character >> 6) & 0x3f));
		p_dest[2] = char(0x80 | (n_character & 0x3f));
		// save as trinity of values
	} else if(n_character <= 0x10ffff) {
		if(n_space != 4)
			return false;
		// make sure there's enough space

		p_dest[0] = char(0xf0 | (n_character >> 18));
		p_dest[1] = char(0x80 | ((n_character >> 12) & 0x3f));
		p_dest[2] = char(0x80 | ((n_character >> 6) & 0x3f));
		p_dest[3] = char(0x80 | (n_character & 0x3f));
		// save as quadruple of values

	} else {
		return false;
		// too high character to encode
	}

	return true;
}

/*
 *								=== ~CUniConv ===
 */

/*
 *								=== CUnicodeMapping::CUnicodeMappingTable ===
 */

CUnicodeMapping::CUnicodeMappingTable::CUnicodeMappingTable()
	:m_n_map_size(0)
{}

CUnicodeMapping::CUnicodeMappingTable::~CUnicodeMappingTable()
{
	for(size_t i = 0; i < m_n_map_size; ++ i) {
		if(m_p_map[i].p_s_name)
			delete[] (char*)m_p_map[i].p_s_name;
	}
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Name() const
{
	return m_s_name;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Unicode_Version() const
{
	return m_s_unicode_ver;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Table_Version() const
{
	return m_s_version;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Format() const
{
	return m_s_format;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Date() const
{
	return m_s_date;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_First_Author() const
{
	return m_s_author;
}

const std::string &CUnicodeMapping::CUnicodeMappingTable::s_Notice() const
{
	return m_s_notice;
}

bool CUnicodeMapping::CUnicodeMappingTable::Load(const char *p_s_filename)
{
	for(size_t i = 0; i < m_n_map_size; ++ i) {
		if(m_p_map[i].p_s_name)
			delete[] (char*)m_p_map[i].p_s_name;
	}
	m_n_map_size = 0;
	m_s_name.clear();
	m_s_unicode_ver.clear();
	m_s_version.clear();
	m_s_format.clear();
	m_s_date.clear();
	m_s_author.clear();
	m_s_notice.clear();
	// cleanup first

	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "r"))
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "r")))
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open file

	std::string s_line;
	bool b_initial_comment_block = true;
	while(!feof(p_fr)) {
		if(!stl_ut::ReadLine(s_line, p_fr)) {
			fclose(p_fr);
			return false;
		}
		size_t b = 0, e = s_line.length();

		if(s_line.find("0x") != 0) {
			if(!s_line.empty() && s_line[0] == '#') {
				if(b_initial_comment_block) {
					try {
						m_s_notice += s_line;
						m_s_notice += '\n';
						// read the initial notice (it states it must accompany the data in any distribution form)

						++ b;
						// skip '#'

						while(b < e && isspace((uint8_t)s_line[b]))
							++ b;
						// skip whitespace

						struct TLabelTable {
							const char *p_s_label;
							size_t n_label_length;
							std::string *p_dest;
						} p_label_table[] = {
							{"Name:", 5, &m_s_name},
							{"Unicode version:", 16, &m_s_unicode_ver},
							{"Table version:",14 , &m_s_version},
							{"Table format:", 13, &m_s_format},
							{"Date:", 5, &m_s_date},
							{"Authors:", 8, &m_s_author}
						};
						const size_t n_label_num = sizeof(p_label_table) / sizeof(p_label_table[0]);

						for(size_t i = 0; i < n_label_num; ++ i) {
							_ASSERTE(strlen(p_label_table[i].p_s_label) == p_label_table[i].n_label_length);
							if(e - b > p_label_table[i].n_label_length && !memcmp(s_line.data() + b,
							   p_label_table[i].p_s_label, p_label_table[i].n_label_length * sizeof(char))) {
								b += p_label_table[i].n_label_length;
								// skip the label

								while(b < e && isspace((uint8_t)s_line[b]))
									++ b;
								// skip space

								std::string &r_s_dest = *p_label_table[i].p_dest;
								r_s_dest.insert(r_s_dest.begin(), s_line.begin() + b, s_line.end());
								// put the rest of the line to the destination string
							}
						}
						// parse labels
					} catch(std::bad_alloc&) {
						return false;
					}
				}
			} else
				b_initial_comment_block = false;
			continue;
		} else
			b_initial_comment_block = false;
		b += 2;
		// skip 0x

		uint32_t n_char = 0;
		while(b < e && isxdigit((uint8_t)s_line[b])) {
			int n_digit = isdigit((uint8_t)s_line[b])? s_line[b] - '0' :
				tolower((uint8_t)s_line[b]) - 'a' + 10;
			n_char <<= 4;
			n_char |= n_digit;
			++ b;
		}
		// convert hexadecimal char code

		if(n_char > 255) {
			fclose(p_fr);
			return false;
		}
		// eg. in SHIFTJIS.TXT

		if(b == e || !isspace((uint8_t)s_line[b]))
			continue;
		while(b < e && isspace((uint8_t)s_line[b]))
			++ b;
		// skip whitespace

		if(s_line.find("0x", b) != b)
			continue;
		b += 2;
		// skip 0x

		int n_unicode = 0;
		while(b < e && isxdigit((uint8_t)s_line[b])) {
			int n_digit = isdigit((uint8_t)s_line[b])? s_line[b] - '0' :
				tolower(s_line[b]) - 'a' + 10;
			n_unicode <<= 4;
			n_unicode |= n_digit;
			++ b;
		}
		// convert hexadecimal unicode

		while(b < e && isspace((uint8_t)s_line[b]))
			++ b;
		// skip whitespace

		if(m_n_map_size == max_Table_Size) {
			fclose(p_fr);
			return false;
		}
		TCharacterName &r_t_new_char = m_p_map[m_n_map_size];
		++ m_n_map_size;
		// put a new element to he map

		r_t_new_char.n_char8 = (uint8_t)n_char;
		r_t_new_char.n_unicode = n_unicode;
		// write characters

		if(b != e && s_line[b] == '#') {
			++ b;
			// skip '#'

			while(b < e && isspace((uint8_t)s_line[b]))
				++ b;
			// skip whitespace

#if defined(_MSC_VER) && !defined(__MWERKS__)
			if(!(r_t_new_char.p_s_name = _strdup(s_line.c_str() + b))) {
#else // _MSC_VER && !__MWERKS__
			if(!(r_t_new_char.p_s_name = strdup(s_line.c_str() + b))) {
#endif // _MSC_VER && !__MWERKS__
				fclose(p_fr);
				return false;
			}
			// copy the comment string
		} else
			r_t_new_char.p_s_name = 0; // no comment
		// write the comment string
	}
	// read the entire file, line by line

	if(ferror(p_fr)) {
		fclose(p_fr);
		return false;
	}
	fclose(p_fr);
	// close the file

	return true;
}

/*
 *								=== ~CUnicodeMapping::CUnicodeMappingTable ===
 */

/*
 *								=== CUnicodeMapping ===
 */

bool CUnicodeMapping::FromTable(const TCharacterName *p_table, size_t n_table_size, bool b_avoid_accents)
{
	m_n_inverse_map_size = 0;

	for(int i = 0; i < 256; ++ i)
		m_p_mapping[i] = -1;
	// clear inverse mapping table

	for(size_t i = 0; i < n_table_size; ++ i) {
		uint8_t n_char = p_table[i].n_char8;
		int n_unicode = p_table[i].n_unicode;

		m_p_mapping[(unsigned char)(n_char)] = n_unicode;
		// store inverse mapping

		if(b_avoid_accents) {
			char n_char_name;
			bool b_capital;
			if(Parse_LatinCharacterName(p_table[i].p_s_name, n_char_name, b_capital)) {
				if(b_capital)
					n_char = toupper(uint8_t(n_char_name));
				else
					n_char = tolower(uint8_t(n_char_name));
				// replace character (possibly with accent) by a simple character
			}
		}
		// try to avoid accents

		if(m_n_inverse_map_size == 256) {
			m_n_inverse_map_size = 0; // to mark error
			return false;
		}
		// too much characters in there

		m_p_inverse_map[m_n_inverse_map_size].n_character = n_char;
		m_p_inverse_map[m_n_inverse_map_size].n_unicode = n_unicode;
		++ m_n_inverse_map_size;
		// add to the list
	}
	// read lines

	std::sort(m_p_inverse_map, m_p_inverse_map + m_n_inverse_map_size, b_SmallerUnicode);
	// sort by unicode

	return true;
}

CUnicodeMapping::CUnicodeMapping(const TCharacterName *p_table, size_t n_table_size, bool b_avoid_accents)
	:m_n_inverse_map_size(0), m_n_subst_char(-1)
{
	if(!FromTable(p_table, n_table_size, b_avoid_accents)) {
		m_n_inverse_map_size = 0; // to mark error
		return;
	}
}

CUnicodeMapping::CUnicodeMapping(const char *p_s_filename, bool b_avoid_accents)
	:m_n_inverse_map_size(0), m_n_subst_char(-1)
{
	CUnicodeMappingTable umt;
	if(!umt.Load(p_s_filename)) {
		m_n_inverse_map_size = 0; // to mark error
		return;
	}
	// load the mapping table from a file

	if(!FromTable(umt.p_Table(), umt.n_Table_Size(), b_avoid_accents)) {
		m_n_inverse_map_size = 0; // to mark error
		return;
	}
	// get the mapping from the table

	/*std::string s_line;
	while(GetLine(s_line, p_fr)) {
		size_t b = 0, e = s_line.length();

		if(s_line.find("0x") != 0)
			continue;
		b += 2;
		// skip 0x

		char n_char = 0;
		while(b < e && isxdigit(s_line[b])) {
			int n_digit = isdigit(s_line[b])? s_line[b] - '0' :
				tolower(s_line[b]) - 'a' + 10;
			n_char <<= 4;
			n_char |= n_digit;
			++ b;
		}
		// convert hexadecimal char code

		if(b == e || !isspace(s_line[b]))
			continue;
		while(b < e && isspace(s_line[b]))
			++ b;
		// skip whitespace

		if(s_line.find("0x", b) != b)
			continue;
		b += 2;
		// skip 0x

		int n_unicode = 0;
		while(b < e && isxdigit(s_line[b])) {
			int n_digit = isdigit(s_line[b])? s_line[b] - '0' :
				tolower(s_line[b]) - 'a' + 10;
			n_unicode <<= 4;
			n_unicode |= n_digit;
			++ b;
		}
		// convert hexadecimal unicode

		while(b < e && isspace(s_line[b]))
			++ b;
		// skip whitespace

		m_p_mapping[(unsigned char)(n_char)] = n_unicode;
		// store inverse mapping

		if(b_avoid_accents) {
			char n_char_name;
			bool b_capital;
			if(Parse_LatinCharacterName(s_line, n_char_name, b_capital)) {
				if(b_capital)
					n_char = toupper(n_char_name);
				else
					n_char = tolower(n_char_name);
				// replace character (possibly with accent) by a simple character
			}
		}
		// try to avoid accents

		if(m_n_inverse_map_size == 256) {
			m_n_inverse_map_size = 0; // to mark error
			fclose(p_fr);
			return;
		}
		// too much characters in there

		m_p_inverse_map[m_n_inverse_map_size].n_character = n_char;
		m_p_inverse_map[m_n_inverse_map_size].n_unicode = n_unicode;
		++ m_n_inverse_map_size;
		// add to the list
	}
	// read lines

	std::sort(m_p_inverse_map, m_p_inverse_map + m_n_inverse_map_size, b_SmallerUnicode);
	// sort by unicode

	if(ferror(p_fr))
		m_n_inverse_map_size = 0; // to mark error
	fclose(p_fr);
	// close file*/
	// not needed anymore
}

bool CUnicodeMapping::b_Status() const
{
	return m_n_inverse_map_size > 0;
}

int CUnicodeMapping::n_FromUnicode(int n_unicode) const
{
	const TCharacterMapping *p_mapping = std::lower_bound(m_p_inverse_map,
		m_p_inverse_map + m_n_inverse_map_size, n_unicode);
	if(p_mapping != m_p_inverse_map + m_n_inverse_map_size)
		return p_mapping->n_character;
	return m_n_subst_char;
}

int CUnicodeMapping::n_FromUnicode(int n_unicode, int n_substitute) const
{
	const TCharacterMapping *p_mapping = std::lower_bound(m_p_inverse_map,
		m_p_inverse_map + m_n_inverse_map_size, n_unicode);
	if(p_mapping != m_p_inverse_map + m_n_inverse_map_size)
		return p_mapping->n_character;
	return n_substitute;
}

int CUnicodeMapping::n_Set_SubsituteChar(int n_substitute)
{
	int n_result = m_n_subst_char;
	m_n_subst_char = n_substitute;
	return n_result;
}

int CUnicodeMapping::n_SubsituteChar() const
{
	return m_n_subst_char;
}

bool CUnicodeMapping::FromUnicode(std::string &r_s_dest,
	const std::basic_string<int> &r_s_string, char n_substitute) const
{
	if(!stl_ut::Resize_To_N(r_s_dest, r_s_string.length()))
		return false;
	// alloc dest string

	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i)
		r_s_dest[i] = n_FromUnicode(r_s_string[i], n_substitute);
	// translate

	return true;
}

bool CUnicodeMapping::ToUnicode(std::basic_string<int> &r_s_dest, std::string &r_s_string)
{
	if(!stl_ut::Resize_To_N(r_s_dest, r_s_string.length()))
		return false;
	// alloc dest string

	for(size_t i = 0, n = r_s_string.length(); i < n; ++ i) {
		int n_code;
		if((n_code = n_ToUnicode(r_s_string[i])) == -1)
			return false;
		r_s_dest[i] = n_code;
	}
	// translate

	return true;
}

inline bool CUnicodeMapping::b_HigherUnicode(const TCharacterMapping &r_t_a,
	int n_unicode)
{
	return r_t_a.n_unicode < n_unicode;
}

inline bool CUnicodeMapping::b_SmallerUnicode(const TCharacterMapping &r_t_a,
	const TCharacterMapping &r_t_b)
{
	return r_t_a.n_unicode < r_t_b.n_unicode;
}

/*bool CUnicodeMapping::GetLine(std::string &r_s_line, FILE *p_fr)
{
	while(!feof(p_fr)) {
		r_s_line.erase();
		for(int c = fgetc(p_fr); c != '\n' && c != EOF; c = fgetc(p_fr)) {
			if(!stl_ut::Reserve_1More(r_s_line))
				return false;
			r_s_line += c;
		}
		// read line

		if(r_s_line.find('#') == 0)
			r_s_line.erase(r_s_line.find('#'));
		// throw away full-line comment

		size_t b = 0, e = r_s_line.length();
		while(e > 0 && isspace(r_s_line[e - 1]))
			-- e;
		while(b < e && isspace(r_s_line[b]))
			++ b;
		r_s_line.erase(e);
		r_s_line.erase(0, b);
		// throw away begin / end whitespace

		if(r_s_line.empty())
			continue;
		// skip empty lines

		return true;
	}

	return false;
}*/

bool CUnicodeMapping::Parse_LatinCharacterName(const char *p_s_line,
	char &r_n_char_name, bool &r_b_capital)
{
	size_t b = 0, e = strlen(p_s_line);
	if(b == e)
		return false;

	while(b < e && isspace(p_s_line[b]))
		++ b;
	// skip whitespace

	const char *p_s_latin = "latin";
	while(b < e && tolower(p_s_line[b]) == *p_s_latin) {
		++ p_s_latin;
		++ b;
	}
	if(*p_s_latin)
		return false;
	// skip "latin"

	if(b == e || !isspace(p_s_line[b]))
		return false;
	while(b < e && isspace(p_s_line[b]))
		++ b;
	// skip whitespace

	bool b_capital;
	const char *p_s_case;
	if(b < e && tolower(p_s_line[b]) == 'c') {
		p_s_case = "capital";
		b_capital = true;
	} else if(b < e && tolower(p_s_line[b]) == 's') {
		p_s_case = "small";
		b_capital = false;
	} else
		return false;
	while(b < e && tolower(p_s_line[b]) == *p_s_case) {
		++ p_s_case;
		++ b;
	}
	if(*p_s_case)
		return false;
	// skip "capital" or "small"

	if(b == e || !isspace(p_s_line[b]))
		return false;
	while(b < e && isspace(p_s_line[b]))
		++ b;
	// skip whitespace

	const char *p_s_letter = "letter";
	while(b < e && tolower(p_s_line[b]) == *p_s_letter) {
		++ p_s_letter;
		++ b;
	}
	if(*p_s_letter)
		return false;
	// skip "letter"

	if(b == e || !isspace(p_s_line[b]))
		return false;
	while(b < e && isspace(p_s_line[b]))
		++ b;
	// skip whitespace

	if(b < e && isalpha(p_s_line[b]) &&
	   ((b + 1 < e && isspace(p_s_line[b + 1])) || b + 1 == e)) {
		r_n_char_name = p_s_line[b];
		r_b_capital = b_capital;
		return true;
	}
	// in case there's a single letter

	return false;
}

/*
 *								=== ~CUnicodeMapping ===
 */
