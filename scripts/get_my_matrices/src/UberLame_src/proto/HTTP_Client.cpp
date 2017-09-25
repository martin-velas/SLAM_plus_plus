/*
								+---------------------------------+
								|                                 |
								|  ***   Simple HTTP client  ***  |
								|                                 |
								|  Copyright  © -tHE SWINe- 2007  |
								|                                 |
								|         HTTP_Client.cpp         |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file proto/HTTP_Client.cpp
 *	@date 2007
 *	@author -tHE SWINe-
 *	@brief Simple HTTP client
 *
 *	@date 2010-02-12
 *
 *	revised source code, fixed some minor issues with memory allocation checks, minor clean-ups
 *
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include <string>
#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "../Socket.h"
#include "HTTP_Client.h"

/*
 *								=== CHTTP_Traffic ===
 */

uint64_t CHTTP_Traffic::n_traffic_up = 0;
uint64_t CHTTP_Traffic::n_traffic_down = 0;

/*
 *								=== ~CHTTP_Traffic ===
 */

/*
 *								=== CHTTP_Socket ===
 */

CHTTP_Socket::CHTTP_Socket(size_t n_buffer_size, int n_timeout)
	:m_n_buffer_size(n_buffer_size)
{
	_ASSERTE(sizeof(char) == 1);
	m_p_buffer_begin = new(std::nothrow) char[n_buffer_size];
	m_p_buffer_ptr = m_p_buffer_end = m_p_buffer_begin + n_buffer_size;

	if(n_timeout > 0)
		CSocket::SetTimeout(n_timeout);
}

CHTTP_Socket::~CHTTP_Socket()
{
	if(m_p_buffer_begin)
		delete[] m_p_buffer_begin;
}

bool CHTTP_Socket::b_Status() const
{
	return m_p_buffer_begin != 0 && CStreamSocket::b_Status();
}

CHTTP_Socket *CHTTP_Socket::p_Accept(TNetAddress &r_t_address, size_t n_buffer_size, int n_timeout) const
{
#if defined(_WIN32) || defined(_WIN64)
	SOCKET n_socket;
	int n_addr_len = sizeof(struct sockaddr);
	if((n_socket = accept(m_n_socket, (sockaddr*)&r_t_address.t_Address(),
	   &n_addr_len)) == INVALID_SOCKET)
		return 0;
	// maybe just timeout or something ... do not disconnect me yet

	CHTTP_Socket *p_new;
	if(!(p_new = new(std::nothrow) CHTTP_Socket(n_buffer_size, n_timeout)))
		return 0;
	if(p_new->b_Status())
		closesocket(p_new->m_n_socket); // close it right away
	p_new->m_n_socket = n_socket;
	// assign socket

	return p_new;
#else // _WIN32 || _WIN64
	int n_socket;
	unsigned int n_length = sizeof(struct sockaddr_in);
	if((n_socket = accept(m_n_socket, (struct sockaddr*)&r_t_address.t_Address(),
	   &n_length)) < 0)
		return 0;
	// maybe just timeout or something ... do not disconnect me yet

	CHTTP_Socket *p_new;
	if(!(p_new = new(std::nothrow) CHTTP_Socket(n_buffer_size)))
		return 0;
	if(p_new->b_Status())
		close(p_new->m_n_socket); // close it right away
	p_new->m_n_socket = n_socket;
	// assign socket

	return p_new;
#endif // _WIN32 || _WIN64
}

int CHTTP_Socket::n_Read(void *p_buffer, size_t n_buffer_size)
{
	char *_p_buffer = (char*)p_buffer;

	size_t n_read_total = 0;
	for(size_t n_remains = n_buffer_size; n_remains;) {
		if(m_p_buffer_end == m_p_buffer_ptr) {
			m_p_buffer_ptr = m_p_buffer_begin;
			m_p_buffer_end = m_p_buffer_begin; // buffer must remain empty in case of error
			int n_read;
			if((n_read = CStreamSocket::n_Read(m_p_buffer_begin, m_n_buffer_size)) < 0) {
				_ASSERTE(n_read_total < INT_MAX);
				return b_WouldBlock()? int(n_read_total) : n_read;
			}
			if(!n_read) {
				_ASSERTE(n_read_total < INT_MAX);
				return int(n_read_total);
			}
			CHTTP_Traffic::Traffic_Down(n_read);
			m_p_buffer_end = m_p_buffer_begin + n_read;
		}
		// makes sure buffer is not empty

		size_t n_have = m_p_buffer_end - m_p_buffer_ptr;
		if(n_have > n_remains)
			n_have = n_remains;		
		memcpy(_p_buffer, m_p_buffer_ptr, n_have);
		m_p_buffer_ptr += n_have;
		_p_buffer += n_have;
		n_remains -= n_have;
		n_read_total += n_have;
		// reads data
	}
	_ASSERTE(n_read_total < INT_MAX);
	return int(n_read_total);
}

bool CHTTP_Socket::ReadMessage(std::string &r_s_output, const char *p_s_delimiter)
{
	_ASSERTE(p_s_delimiter && *p_s_delimiter); // delimiter must not be empty

	r_s_output.erase(r_s_output.begin(), r_s_output.end());
	// clean output string

	char n_last_delim_char = p_s_delimiter[strlen(p_s_delimiter) - 1];
	size_t n_delim_len = strlen(p_s_delimiter);
	// remember last delimiter character

	for(;;) {
		if(m_p_buffer_end == m_p_buffer_ptr) {
			m_p_buffer_ptr = m_p_buffer_begin;
			int n_read;
			if((n_read = CStreamSocket::n_Read(m_p_buffer_begin, m_n_buffer_size)) <= 0)
				return false;
			CHTTP_Traffic::Traffic_Down(n_read);
			m_p_buffer_end = m_p_buffer_begin + n_read;
		}
		// makes sure buffer is not empty

		for(;m_p_buffer_ptr != m_p_buffer_end; ++ m_p_buffer_ptr) {
			try {
				r_s_output += *m_p_buffer_ptr;
				// append character
			} catch(std::bad_alloc&) {
				return false;
				// not enough memory
			}

			if(*m_p_buffer_ptr == n_last_delim_char && r_s_output.length() >= n_delim_len &&
			   !strcmp(r_s_output.c_str() + (r_s_output.length() - n_delim_len), p_s_delimiter)) {
				++ m_p_buffer_ptr; // !!
				return true;
			}
			// see if string contains delimiter
		}
		// append characters until delimiter is contained within the string
	}
}

/*
 *								=== ~CHTTP_Socket ===
 */

/*
HTTP/1.1 header field values can be folded onto multiple lines if the continuation line
begins with a space or horizontal tab. All linear white space, including folding, has
the same semantics as SP. A recipient MAY replace any linear white space with a single
SP before interpreting the field value or forwarding the message downstream. 

       generic-message = start-line
                         *(message-header CRLF)
                         CRLF
                         [ message-body ]
       start-line      = Request-Line | Status-Line

		   Status-Line = HTTP-Version SP Status-Code SP Reason-Phrase CRLF

		HTTP-Version   = "HTTP" "/" 1*DIGIT "." 1*DIGIT

	    message-header = field-name ":" [ field-value ]
	    field-name     = token
	    field-value    = *( field-content | LWS )
	    field-content  = <the OCTETs making up the field-value
						 and consisting of either *TEXT or combinations
						 of token, separators, and quoted-string>

1.Any response message which "MUST NOT" include a message-body (such as the 1xx, 204,
and 304 responses and any response to a HEAD request) is always terminated by the first
empty line after the header fields, regardless of the entity-header fields present in
the message. 

2.If a Transfer-Encoding header field (section 14.41) is present and has any value other
than "identity", then the transfer-length is defined by use of the "chunked" transfer-coding
(section 3.6), unless the message is terminated by closing the connection. 

3.If a Content-Length header field (section 14.13) is present, its decimal value in OCTETs
represents both the entity-length and the transfer-length. The Content-Length header field
MUST NOT be sent if these two lengths are different (i.e., if a Transfer-Encoding 

4.If the message uses the media type "multipart/byteranges", and the ransfer-length is not
otherwise specified, then this self- elimiting media type defines the transfer-length. This
media type UST NOT be used unless the sender knows that the recipient can arse it; the presence
in a request of a Range header with ultiple byte- range specifiers from a 1.1 client implies
that the lient can parse multipart/byteranges responses. 

If the message does include a non- identity transfer-coding, the Content-Length MUST be ignored. 

http://www.w3.org/Protocols/rfc2616/rfc2616-sec14.htm

*/

/*
 *								=== CHTTP_Header ===
 */

#define isTokenChar(c) ((unsigned char)(c) > 33 && (unsigned char)(c) < 127 && \
	(c) != '(' && (c) != ')' && (c) != '<' && (c) != '>' && (c) != '@' && (c) != ',' && \
	(c) != ';' && (c) != ':' && (c) != '\\' && (c) != '\"' && (c) != '/' && (c) != '[' && \
	(c) != ']' && (c) != '?' && (c) != '=' && (c) != '{' && (c) != '}')

CHTTP_Header::CHTTP_Header()
	:m_n_status_code(0), m_b_response_type(true)
{}

CHTTP_Header::CHTTP_Header(const char *p_s_message, size_t &r_n_message_length)
	:m_n_status_code(0)
{
	if(!Parse(p_s_message, r_n_message_length))
		r_n_message_length = 0;
}

bool CHTTP_Header::Parse(const char *p_s_message, size_t &r_n_message_length)
{
	const char *p_s_message_ptr;
	if(m_s_status_line.empty()) {
		const char *p_s_status_end;
		if(!(p_s_status_end = strstr(p_s_message, "\r\n")))
			return false;
		if(!stl_ut::Reserve_N(m_s_status_line, p_s_status_end - p_s_message))
			return false;
		m_s_status_line.insert(m_s_status_line.begin(), p_s_message, p_s_status_end);
		// parse status line

		if(!strncmp(p_s_message, "HTTP/", 5 /*strlen("HTTP/")*/)) {
			m_b_response_type = true;
			// this is server response type message

			_ASSERTE(strstr(p_s_message, "HTTP/") == p_s_message); // make sure it's HTTP response
			if((p_s_message_ptr = strchr(p_s_message, ' ')) > p_s_status_end || !p_s_message_ptr)
				return false;
			m_n_status_code = atol(p_s_message_ptr);
			// skip to status code and parse it
		} else {
			m_b_response_type = false;
			// this is client request type message

			const char *p_s_method_end = p_s_message;
			while(isalpha(*p_s_method_end))
				++ p_s_method_end;
			// find the method field end (wont pass end of line)

			m_s_method.erase();
			if(!stl_ut::Reserve_N(m_s_method, p_s_method_end - p_s_message))
				return false;
			m_s_method.insert(m_s_method.begin(), p_s_message, p_s_method_end);
			// insert method into it's own string

			const char *p_s_value_begin = p_s_method_end + 1;
			while(isspace(*p_s_value_begin) && p_s_value_begin < p_s_status_end)
				++ p_s_value_begin;
			if(p_s_value_begin == p_s_status_end)
				return false;
			// find the method field end (could pass end of line)

			const char *p_s_value_end = p_s_value_begin + 1;
			while(!isspace(*p_s_value_end))
				++ p_s_value_end;
			// find value end (won't pass end of line)

			m_s_request_uri.erase();
			if(!stl_ut::Reserve_N(m_s_request_uri, p_s_value_end - p_s_value_begin))
				return false;
			m_s_request_uri.insert(m_s_request_uri.begin(), p_s_value_begin, p_s_value_end);
			// insert request URI into it's own string
		}

		p_s_message_ptr = p_s_status_end + 2;
		// skip past status line
	} else
		p_s_message_ptr = p_s_message;
	// in case it's called after once parsed, doesn't expect status line, just appends fields

	return Parse_Fields(p_s_message, p_s_message_ptr, r_n_message_length);
}

bool CHTTP_Header::Parse_Fields(const char *p_s_message, const char *p_s_message_ptr,
	size_t &r_n_message_length)
{
	for(;;) {
		const char *p_s_line_end;
		if(!(p_s_line_end = strstr(p_s_message_ptr, "\r\n")))
			return false;
		if(p_s_line_end == p_s_message_ptr)
			break;
		// find end of line, in case there are two endlines
		// immediately next to each other, it's the end of message

		const char *p_s_token_begin = p_s_message_ptr;
		_ASSERTE(isTokenChar(*p_s_token_begin));
		const char *p_s_token_end = p_s_message_ptr;
		while(isTokenChar(*p_s_token_end))
			++ p_s_token_end;
		// find field name token end (wont pass line end)

		const char *p_s_value_begin;
		_ASSERTE(*p_s_token_end == ':'); // should be colon (but specs say whitespace might follow)
		if(!(p_s_value_begin = strchr(p_s_token_end, ':')))
			return false;
		++ p_s_value_begin;
		while(isspace(*p_s_value_begin) && p_s_value_begin < p_s_line_end)
			++ p_s_value_begin;
		if(p_s_value_begin == p_s_line_end)
			return false;
		// find value begin (could pass line end)

		const char *p_s_value_end = p_s_line_end;
		while(*(p_s_value_end - 1) == ' ' || *(p_s_value_end - 1) == '\t')
			-- p_s_value_end;
		_ASSERTE(p_s_value_end > p_s_value_begin);
		// find value end

		std::string s_value;
		if(!stl_ut::Reserve_N(s_value, p_s_value_end - p_s_value_begin))
			return false;
		s_value.insert(s_value.begin(), p_s_value_begin, p_s_value_end);
		// create string, containing value

		while(*(p_s_line_end + 2) == ' ' || *(p_s_line_end + 2) == '\t') {
			p_s_message_ptr = p_s_line_end + 3;
			if(!(p_s_line_end = strstr(p_s_message_ptr, "\r\n")))
				return false;
			_ASSERTE(p_s_line_end > p_s_message_ptr);
			// find next line end

			const char *p_s_fold_value_begin = p_s_message_ptr;
			while(isspace(*p_s_fold_value_begin) && p_s_fold_value_begin < p_s_line_end)
				++ p_s_fold_value_begin;
			if(p_s_fold_value_begin == p_s_line_end)
				break; // nothing follows
			// find folded value begin

			const char *p_s_fold_value_end = p_s_line_end;
			while(*(p_s_fold_value_end - 1) == ' ' || *(p_s_fold_value_end - 1) == '\t')
				-- p_s_fold_value_end;
			_ASSERTE(p_s_fold_value_end > p_s_fold_value_begin);
			// find folded value end

			if(!stl_ut::Reserve_NMore(s_value, p_s_fold_value_end - p_s_fold_value_begin + 1))
				return false;
			s_value += ' ';
			s_value.insert(s_value.end(), p_s_fold_value_begin, p_s_fold_value_end);
			// append folded value, separate it with single space
		}
		// append possible folded values

		if(!Add_Field(p_s_token_begin, p_s_token_end, s_value.c_str()))
			return false;
		// add parsed field into the list

		p_s_message_ptr = p_s_line_end + 2;
		// skip to next line
	}

	r_n_message_length = (size_t)(p_s_message_ptr - p_s_message + 2);
	// calculate total length of parsed message

	return true;
}

const char *CHTTP_Header::p_s_Field(const char *p_s_name) const
{
	std::vector<TField>::const_iterator p_field_iter;
	if((p_field_iter = std::find(m_field_list.begin(), m_field_list.end(),
	   p_s_name)) != m_field_list.end())
		return (*p_field_iter).s_value.c_str();
	return 0;
}

bool CHTTP_Header::Add_Field(const char *p_s_name, const char *p_s_name_end, const char *p_s_value)
{
	if(!stl_ut::Resize_Add_1More(m_field_list))
		return false;
	TField &r_field = m_field_list.back();
	// add one more field

	if(!stl_ut::Reserve_N(r_field.s_name, p_s_name_end - p_s_name) ||
	   !stl_ut::AssignCStr(r_field.s_value, p_s_value)) {
		m_field_list.erase(m_field_list.end() - 1);
	    return false;
	}
	// allocate strings directly in referenced field in the
	// list to avoid unnecessary string copying

	r_field.s_name.insert(r_field.s_name.begin(), p_s_name, p_s_name_end);

	return true;
}

/*
 *								=== ~CHTTP_Header ===
 */

/*
 *								=== CHTTP_File ===
 */

CHTTP_File::CHTTP_File(const char *p_s_location)
	:m_p_header(0), m_n_size(0)
{
	stl_ut::AssignCStr(m_s_location, p_s_location);
}

CHTTP_File::~CHTTP_File()
{
	if(m_p_header)
		delete m_p_header;
}

bool CHTTP_File::Connect_SendGET(const char *p_s_proxy,
	CHTTP_Socket &r_socket, bool b_use_post, const char *p_s_request_params)
{
	std::string s_hostname;
	s_hostname = m_s_location;
	if(s_hostname.find("://") != std::string::npos) {
		s_hostname.erase(s_hostname.begin(), s_hostname.begin() +
			(s_hostname.find("://") + 3));
	}
	if(s_hostname.find('/') != std::string::npos)
		s_hostname.erase(s_hostname.begin() + s_hostname.find('/'), s_hostname.end());
	// create hostname

	std::string s_request;
	std::string s_gateway_hostname;
	int n_gateway_port = 80;
	__STL_UT_TRY {
		s_request = (b_use_post)? "POST " : "GET ";
		if(m_s_location.find("http://") != 0)
			s_request += "http://"; // there must be "http://" otherwise most proxies won't work !!
		s_request += m_s_location;
		s_request += " HTTP/1.1\r\n";
		//s_request += "Accept: text/html\r\n";
		s_request += "Pragma: no-cache\r\n";
		s_request += "Host: ";
		s_request += s_hostname;
		s_request += "\r\n";
		s_request += "Proxy-Connection: close, TE\r\n";
		s_request += "TE: chunked, identity\r\n";
		if(p_s_request_params)
			s_request += p_s_request_params;
		s_request += "\r\n"; // maybe there's one too many, but nevermind (t_odo - find out)
		// form simple http gateway request

		s_gateway_hostname = p_s_proxy;
		if(s_gateway_hostname.find(':') != std::string::npos) {
			n_gateway_port = atol(s_gateway_hostname.c_str() + s_gateway_hostname.find(':') + 1);
			s_gateway_hostname.erase(s_gateway_hostname.begin() +
				s_gateway_hostname.find(':'), s_gateway_hostname.end());
		}
		// split proxy address to hostname and port
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// memory allocation error
	}

	if(!r_socket.b_Status() || !r_socket.Connect(TNetAddress(s_gateway_hostname.c_str(), n_gateway_port)))
		return false;
	// create buffered socket and connect to host

	CHTTP_Traffic::Traffic_Up(s_request.length());
	if(r_socket.n_Write(s_request.data(), s_request.length()) != (signed)s_request.length())
		return false;
	// send request

	return true;
}

bool CHTTP_File::Connect_SendGET(CHTTP_Socket &r_socket, bool b_use_post, const char *p_s_request_params)
{
	std::string s_request;
	std::string s_hostname;
	std::string s_filename;
	int n_port = 80;

	__STL_UT_TRY {
		s_hostname = m_s_location;
		if(s_hostname.find("://") != std::string::npos) {
			s_hostname.erase(s_hostname.begin(), s_hostname.begin() +
				(s_hostname.find("://") + 3));
		}
		if(s_hostname.find('/') != std::string::npos) {
			s_filename = s_hostname.substr(s_hostname.find('/'));
			s_hostname.erase(s_hostname.begin() + s_hostname.find('/'), s_hostname.end());
		}
		if(s_hostname.find(':') != std::string::npos) {
			n_port = atol(s_hostname.c_str() + s_hostname.find(':') + 1);
			s_hostname.erase(s_hostname.begin() + s_hostname.find(':'), s_hostname.end());
		}
		// parse file address

		s_request = (b_use_post)? "POST " : "GET ";
		s_request += (!s_filename.empty())? s_filename.c_str() : "/";
		s_request += " HTTP/1.1\r\nHost: ";
		s_request += s_hostname;
		if(n_port != 80) {
			std::string s_port;
			if(!stl_ut::Format(s_port, ":%d", n_port))
				return false;
			s_request += s_port;
			/*char p_s_port[16];
			p_s_port[0] = ':';
			itoa(n_port, p_s_port + 1, 10);*/
			//sprintf(p_s_port, ":%d", n_port);
			//s_request += p_s_port;
		}
		s_request += "\r\n";
		if(p_s_request_params)
			s_request += p_s_request_params;
		if(!b_use_post)
			s_request += "Connection: close\r\n\r\n";
		// create GET / POST request string
	} __STL_UT_CATCH_BAD_ALLOC {
		return false;
		// memory allocation error
	}

	if(!r_socket.b_Status() || !r_socket.Connect(TNetAddress(s_hostname.c_str(), n_port)))
		return false;
	// create buffered socket and connect to host

	CHTTP_Traffic::Traffic_Up(s_request.length());
	if(r_socket.n_Write(s_request.data(), s_request.length()) != (signed)s_request.length())
		return false;
	// send request

	return true;
}

/*
 *								=== ~CHTTP_File ===
 */
