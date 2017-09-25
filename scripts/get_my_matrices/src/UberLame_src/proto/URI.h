/*
								+---------------------------------+
								|                                 |
								|     ***   URI helpers   ***     |
								|                                 |
								|  Copyright  © -tHE SWINe- 2010  |
								|                                 |
								|              URI.h              |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __URI_UTILS_INCLUDED
#define __URI_UTILS_INCLUDED

/**
 *	@file proto/URI.h
 *	@date 2010
 *	@author -tHE SWINe-
 *	@brief URI helpers
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 */

#include "../StlUtils.h"
#include "../UniConv.h"
#include <map>
#include "HTTP_Client.h"

/**
 *	@brief utility functions for request URI transforms
 */
class CURI_Utils {
public:
	/**
	 *	@brief replaces non-URI characters by "% hex hex" escape sequences in UTF8
	 *
	 *	@param[out] r_s_dest is destination string for the URI
	 *	@param[in] r_s_uri is plain URI string (may contain non-us-english characters, which are in turn escaped)
	 *	@param[in] encoder is function object responsible for translating 8-bit characters to unicode
	 *
	 *	@return Returns true on success, false on failure (malformed codes / not enough memory).
	 */
	template <class CEncodeUnicode>
	static bool Escape_URI_UTF8(std::string &r_s_dest, const std::string &r_s_uri, CEncodeUnicode encoder)
	{
		r_s_dest.erase();
		// erase destination string

		for(size_t i = 0, n = r_s_uri.length(); i < n; ++ i) {
			int n_char = encoder(r_s_uri[i]);
			// translate character to unicode

			const char *p_s_gen_delims = ":/?#[]@";
			const char *p_s_sub_delims = "!$&\'()*+,;=";
			const char *p_s_unreserved = "-._~";
			if(n_char < 0 || n_char >= 0x80 || r_s_uri[i] != n_char || (!isdigit(n_char) && !isalpha(n_char) &&
			   !strchr(p_s_gen_delims, n_char) && !strchr(p_s_sub_delims, n_char) &&
			   !strchr(p_s_unreserved, n_char))) {
				std::string s_utf8;
				if(!CUniConv::UTF32_to_UTF8(&n_char, 1, false, true, s_utf8, false))
					return false;
				// translate character to utf-8

				if(!stl_ut::Reserve_NMore(r_s_dest, 3 * s_utf8.length()))
					return false;
				// make space for extra characters in uri

				for(int j = 0, m = s_utf8.length(); j < m; ++ j) {
					std::string s_number;
					if(!stl_ut::Format(s_number, "%%%02x", int((unsigned char)(s_utf8[j]))) ||
					   !stl_ut::Append(r_s_dest, s_number))
						return false;
				}
				// append (escaped) utf-8
			} else {
				if(!stl_ut::Reserve_1More(r_s_dest))
					return false;
				r_s_dest += char(n_char);
				// append plain character (plain 7-bit english)
			}
		}

		return true;
	}

	/**
	 *	@brief replaces escape sequences in "% hex hex" UTF8 format by appropriate characters
	 *
	 *	@param[out] r_s_dest is destination string for the URI
	 *	@param[in] r_string is input URI, containing escape sequences
	 *	@param[in] decoder is function object responsible for translating unicode characters to 8-bit charset
	 *
	 *	@return Returns true on success, false on failure (malformed codes / not enough memory)
	 */
	template <class CDecodeUnicode>
	static bool Unescape_URI_UTF8(std::string &r_s_dest, const std::string &r_string, CDecodeUnicode decoder)
	{
		if(!stl_ut::Assign(r_s_dest, r_string))
			return false;

		for(;;) {
			size_t n_position;
			if((n_position = r_s_dest.find('%')) != std::string::npos) {
				uint8_t n_byte;
				if(!GetByte(r_s_dest, n_position + 1, n_byte))
					return false;
				// get first byte

				int n_byte_num;
				if((n_byte_num = CUniConv::n_UTF8_Char_Size(n_byte)) < 0)
					return false;
				// determine number of bytes

				if(n_byte_num == 1)
					r_s_dest[n_position] = (char)n_byte;
				else {
					uint8_t p_utf8[4];
					p_utf8[0] = n_byte;
					for(int i = 1; i < n_byte_num; ++ i) {
						r_s_dest.erase(n_position + 1, 1); // erase additional '%'
						if(!GetByte(r_s_dest, n_position + 1, p_utf8[i]))
							return false;
					}
					// read additional bytes

					int n_read_bytes;
					int n_character;
					if((n_character = CUniConv::n_UTF8_Code(p_utf8, n_byte_num, n_read_bytes)) < 0)
						return false;
					_ASSERTE(n_byte_num == n_read_bytes);
					// decode utf-8

					r_s_dest[n_position] = decoder(n_character);
					// translate to local charset
				}
			} else
				break;
		}
		// parse %xx character codes

		return true;
	}

	/**
	 *	@brief gets filename along with path, extracted from p_s_uri; handle absolute URI's
	 *
	 *	@param[out] r_s_filename will contaion output filename
	 *	@param[in] r_s_uri is URI of a file (unescaped)
	 *	@param[in] p_s_default is default filename for cases p_s_uri doesn't contain it
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_FileName_Part(std::string &r_s_filename,
		const std::string &r_s_uri, const char *p_s_default = "index.html");

	/**
	 *	@brief removes a named parameter from URI
	 *
	 *	@param[out] r_s_omit_uri is request uri without the specified parameter
	 *		(if any other params follow, they are contained)
	 *	@param[in] p_s_param_name is name of the parameter to be removed
	 *	@param[in] b_erase_leading_slash is optional removal of leading slash flag
	 *		(if set and the uri starts with '/', it is removed)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Remove_URI_Param(std::string &r_s_omit_uri,
		const std::string &r_s_uri, const char *p_s_param_name = "hmac",
		bool b_erase_leading_slash = true)
	{
		if(!stl_ut::Assign(r_s_omit_uri, r_s_uri))
			return false;
		size_t n_param_pos = 0;;
		while((n_param_pos = r_s_uri.find(p_s_param_name, n_param_pos)) != std::string::npos) {
			if(n_param_pos == 0) {
				++ n_param_pos;
				continue;
			}
			if(r_s_uri[n_param_pos - 1] != '&' &&
			   r_s_uri[n_param_pos - 1] != '?') {
			    ++ n_param_pos;
				continue;
			}
			size_t n_term_pos = n_param_pos + strlen(p_s_param_name);
			if(n_term_pos == r_s_uri.length() || r_s_uri[n_term_pos] == '=') {
				// in case it is terminated by the end of string or by a '=', it is it

				r_s_omit_uri.erase(n_param_pos - 1, r_s_uri.find('&', n_param_pos + 1));
				// erase until the next param or until the end if it's the last one

				break;
			} else
				++ n_param_pos;
		}
		if(b_erase_leading_slash && !r_s_omit_uri.empty() && r_s_omit_uri[0] == '/')
			r_s_omit_uri.erase(0, 1);
		// get request uri without the parameter (if any other params follow, they are contained)

		return true;
	}

	/**
	 *	@brief parses uri for parameters
	 *
	 *	Parameters are after the '?' token, are separated by the '&' token
	 *	and have the 'name=value' form. The '?' token is only allowed once.
	 *
	 *	@param[out] r_arg_map is a map of parameters parsed from the URI
	 *	@param[in] r_s_uri is URI of a file (escaped)
	 *	@param[in] b_allow_duplicate_params is duplicate parameters allowance flag
	 *	@param[in] b_allow_flags is value-less parameters allowance flag
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Split_URI_Params(std::map<std::string, std::string> &r_arg_map,
		const std::string &r_s_uri, bool b_allow_duplicate_params, bool b_allow_flags)
	{
		r_arg_map.clear();

		if(r_s_uri.find('?') != std::string::npos) {
			std::vector<std::string> uri_parts, arg_list;
			if(!stl_ut::Split(uri_parts, r_s_uri, "?", 0) || uri_parts.size() != 2 ||
			   !stl_ut::Split(arg_list, uri_parts[1], "&", 0))
				return false;
			// read uri arguments

			try {
				std::string s_value;
				for(size_t i = 0, n = arg_list.size(); i < n; ++ i) {
					s_value.clear();
					size_t n_pos;
					if((n_pos = arg_list[i].find('=')) != std::string::npos) {
						s_value.insert(s_value.begin(), arg_list[i].begin() +
							(n_pos + 1), arg_list[i].end());
						arg_list[i].erase(n_pos);
						// have name=value pair

						if(!b_allow_duplicate_params && r_arg_map.find(arg_list[i]) != r_arg_map.end())
							return false;
						r_arg_map[arg_list[i]] = s_value;
					} else {
						if(!b_allow_flags)
							return false;
						if(!b_allow_duplicate_params && r_arg_map.find(arg_list[i]) != r_arg_map.end())
							return false;
						r_arg_map[arg_list[i]] = "";
					}
				}
			} catch(std::bad_alloc&) {
				return false;
			}
			// convert it to a map
		}
		// parse args // todo - out it to uri utils

		return true;
	}

protected:
	static bool GetByte(std::string &r_string, size_t n_position, uint8_t &r_n_byte);
};

class CURI_Context {
protected:
	const CHTTP_Header *p_header;
	CHTTP_Socket *p_socket;

	TNetAddress t_address;
	std::string s_address;

	bool b_method_get;
	std::string s_str_uri;
	std::string s_file;
	bool b_uri_has_arguments;
	std::map<std::string, std::string> arg_map;

public:
	inline CURI_Context(const CHTTP_Header *_p_header, CHTTP_Socket *_p_socket) // throw(std::bad_alloc)
		:p_header(_p_header), p_socket(_p_socket)
	{
		b_method_get = strcmp(p_header->p_s_Method(), "HEAD") != 0;
		_ASSERTE(!b_method_get || !strcmp(p_header->p_s_Method(), "GET") ||
			!strcmp(p_header->p_s_Method(), "POST"));
		// determine wheter GET/POST or HEAD

		if(!CURI_Utils::Unescape_URI_UTF8(s_str_uri, p_header->s_Request_URI(), n_Unicode_to_US_English))
			throw std::bad_alloc();
		if(!CURI_Utils::Get_FileName_Part(s_file, s_str_uri, "index.html"))
			throw std::bad_alloc();
		// extract desired filename along with path; handle absolute URI's

		p_socket->GetPeerName(t_address);
		if(!t_address.p_s_ToString(s_address))
			throw std::bad_alloc();
		size_t n_pos;
		if((n_pos = s_address.find(':')) != std::string::npos)
			s_address.erase(n_pos);
		// get peer name (for nonces, sessions, ...)

		b_uri_has_arguments = s_str_uri.find('?') != std::string::npos;
		if(!CURI_Utils::Split_URI_Params(arg_map, s_str_uri, false, false))
			throw std::bad_alloc();
		// parse args

		/*if(!CURI_Utils::Remove_URI_Param(s_orig_request_uri, s_str_uri, "hmac", true))
			throw std::bad_alloc();*/
		// get request uri without hmac information (if any args follow, they are contained)
	}

	const std::string &s_URI() const
	{
		return s_str_uri;
	}

	const std::string &s_Filename() const
	{
		return s_file;
	}

	const std::string &s_ClientAddress() const
	{
		return s_address;
	}

	const bool b_HasArgs() const
	{
		return b_uri_has_arguments;
	}

	const bool b_Method_GET() const
	{
		return b_method_get;
	}

	const bool b_Method_HEAD() const
	{
		return !b_method_get;
	}

	const std::string &s_Arg(const std::string &r_s_arg_name) const
	{
		std::map<std::string, std::string>::const_iterator p_field_it = arg_map.find(r_s_arg_name);
		if(p_field_it == arg_map.end()) {
			static const std::string s_empty;
			return s_empty;
		}
		return (*p_field_it).second;
	}

protected:
	static char n_Unicode_to_US_English(unsigned int n_char)
	{
		return (n_char < 128)? n_char : '_'; // don't use a question mark here
	}
};

#endif // !__URI_UTILS_INCLUDED
