/*
								+---------------------------------+
								|                                 |
								|     ***   URI helpers   ***     |
								|                                 |
								|  Copyright  © -tHE SWINe- 2010  |
								|                                 |
								|             URI.cpp             |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file proto/URI.cpp
 *	@date 2010
 *	@author -tHE SWINe-
 *	@brief URI helpers
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include <string>
#include "URI.h"

/*
 *								=== CURI_Utils ===
 */

bool CURI_Utils::Get_FileName_Part(std::string &r_s_filename, const std::string &r_s_uri, const char *p_s_default)
{
	size_t b = 0, e = (r_s_uri.rfind('?') == std::string::npos)?
		r_s_uri.length() : r_s_uri.rfind('?');
	// do not parse after '?'

	size_t n_pos = r_s_uri.find("://");
	if(n_pos < e && n_pos != std::string::npos)
		b = n_pos + 3;
	// skip "://" (after "http")

	n_pos = r_s_uri.find(":", b);
	if(n_pos < e && n_pos != std::string::npos) {
		b = n_pos + 1;
		while(b < e && isdigit(r_s_uri[b]))
			++ b;
	}
	// skip ":[0-9]*" (port specifier)

	n_pos = r_s_uri.find("/", b);
	if(n_pos < e && n_pos != std::string::npos)
		b = n_pos + 1;
	while(b < e && r_s_uri[b] == '/')
		++ b;
	// skip '/'

	if(b == e)
		return stl_ut::AssignCStr(r_s_filename, p_s_default);
	else {
		try {
			r_s_filename.erase();
			r_s_filename.insert(r_s_filename.begin(), r_s_uri.begin() + b, r_s_uri.begin() + e);
		} catch(std::bad_alloc&) {
			return false;
		}
		return true;
	}
	// is there a filename?
}

bool CURI_Utils::GetByte(std::string &r_string, size_t n_position, uint8_t &r_n_byte)
{
	if(/*n_position < 0 ||*/ r_string.length() < /*unsigned*/(n_position) + 2)
		return false;
	// make sure index is valid

	char p_s_code[2] = {r_string[n_position] | 32, r_string[n_position + 1] | 32};
	r_string.erase(n_position, 2);
	// get code, and erase it from from string

	if(p_s_code[0] >= '0' && p_s_code[0] <= '9')
		r_n_byte = (p_s_code[0] - '0');
	else if(p_s_code[0] >= 'a' && p_s_code[0] <= 'f')
		r_n_byte = (p_s_code[0] - 'a' + 10);
	else
		return false;
	// read the first nibble

	r_n_byte <<= 4;
	// shift

	if(p_s_code[1] >= '0' && p_s_code[1] <= '9')
		r_n_byte |= p_s_code[1] - '0';
	else if(p_s_code[1] >= 'a' && p_s_code[1] <= 'f')
		r_n_byte |= p_s_code[1] - 'a' + 10;
	else
		return false;
	// read the second nibble

	return true;
}

/*
 *								=== ~CURI_Utils ===
 */
