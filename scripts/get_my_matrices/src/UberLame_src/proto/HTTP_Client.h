/*
								+---------------------------------+
								|                                 |
								|  ***   Simple HTTP client  ***  |
								|                                 |
								|  Copyright  © -tHE SWINe- 2007  |
								|                                 |
								|          HTTP_Client.h          |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __HTTP_CLIENT_INCLUDED
#define __HTTP_CLIENT_INCLUDED

/**
 *	@file proto/HTTP_Client.h
 *	@brief Simple HTTP client
 *	@author -tHE SWINe-
 *	@date 2007
 *
 *	@note This tends to produce the "First-chance exception 0x000006C5: The tag is invalid."
 *		when running under Win64/WOW64. That is a windows issue, not the issue with the code.
 *
 *	@date 2010-02-12
 *
 *	revised source code, fixed some minor issues with memory allocation checks, minor clean-ups.
 *
 *	@date 2010-09-19
 *
 *	Added documentation comments, code clean-up.
 *
 *	@date 2011-112
 *
 *	Added timeout parameter to CHTTP_File::Receive() and CHTTP_File::ProxiedReceive().
 *	Rewritten CHTTP_File::Receive() to just call CHTTP_File::ProxiedReceive() with no proxy.
 *
 *	@date 2012-06-19
 *
 *	Moved multiple inclusion guard before file documentation comment.
 *
 *	@date 2012-07-16
 *
 *	Changed file size from size_t to uint64_t to enable correct downloading of files larger
 *	than 4 GB that are not sent with chunked encoding (especially where the data receiver
 *	relies on file size information).
 *
 */

#include "../Socket.h"
#include "../Integer.h"
#include "../StlUtils.h"

/**
 *	@brief HTTP traffic stats
 */
class CHTTP_Traffic {
protected:
	static uint64_t n_traffic_up;
	static uint64_t n_traffic_down;

public:
	/**
	 *	@brief resets HTTP traffic counters to 0
	 *
	 *	@note This function is not thread-safe, and should not be
	 *		called while there are running threads, using HTTP sockets.
	 */
	static inline void Reset()
	{
		n_traffic_up = 0;
		n_traffic_down = 0;
	}

	/**
	 *	@brief gets traffic up
	 *	@return Returns number of bytes sent.
	 */
	static inline uint64_t n_Traffic_Up()
	{
		return n_traffic_up;
	}

	/**
	 *	@brief gets traffic down
	 *	@return Returns number of bytes received.
	 */
	static inline uint64_t n_Traffic_Down()
	{
		return n_traffic_down;
	}

	/**
	 *	@brief counts traffic up
	 *
	 *	@param[in] n_amount is number of bytes sent
	 *
	 *	@note This function is not thread-safe, in case multiple threads are
	 *		using HTTP sockets, traffic stats might be slightly inaccurate.
	 */
	static inline void Traffic_Up(size_t n_amount)
	{
		n_traffic_up += n_amount + 40; // 40 extra bytes added by TCP/IP layer
	}

	/**
	 *	@brief counts traffic down
	 *
	 *	@param[in] n_amount is number of bytes received
	 *
	 *	@note This function is not thread-safe, in case multiple threads are
	 *		using HTTP sockets, traffic stats might be slightly inaccurate.
	 */
	static inline void Traffic_Down(size_t n_amount)
	{
		n_traffic_down += n_amount;
	}
};

/**
 *	@brief bufferred socket class with some extended functionality for easier parsing of HTTP (or similar text protocol) response messages
 */
class CHTTP_Socket : public CStreamSocket {
protected:
	char *m_p_buffer_begin;
	char *m_p_buffer_ptr, *m_p_buffer_end;
	size_t m_n_buffer_size;

public:
	/**
	 *	@brief default constructor; allocates socket buffer
	 *
	 *	@param[in] n_buffer_size is socket buffer size
	 *	@param[in] n_timeout is socket timeout in seconds (0 means no timeout)
	 *
	 *	@note This function may fail if there's not enough memory. It's recommended to call b_Status() afterwards.
	 */
	CHTTP_Socket(size_t n_buffer_size, int n_timeout = 30);

	/**
	 *	@brief destructor
	 */
	~CHTTP_Socket();

	/**
	 *	@brief checks constructor success
	 *
	 *	@return Returns true in case socket was successfully created, otherwise false.
	 */
	bool b_Status() const;

	/**
	 *	@brief accepts incoming connections (to be used by HTTP server)
	 *
	 *	@param[out] r_t_address is filled with network address of client, connected to the socket
	 *	@param[in] n_buffer_size is buffer size for the new socket
	 *	@param[in] n_timeout is timeout for the new socket, in seconds (0 means no timeout)
	 *
	 *	@return Returns a new socket connected to the client, or 0 on failure.
	 *
	 *	@note This propably will return 0 in case Listen() was not successfuly called prior calling this function.
	 */
	CHTTP_Socket *p_Accept(TNetAddress &r_t_address, size_t n_buffer_size, int n_timeout) const;

	/**
	 *	@brief gets size of buffered data
	 *	@return Returns size of data, contained in the buffer, in bytes.
	 */
	inline size_t n_Buffered() const
	{
		_ASSERTE(m_p_buffer_end >= m_p_buffer_ptr);
		return (unsigned)(m_p_buffer_end - m_p_buffer_ptr);
	}

	/**
	 *	@brief gets size of data buffer
	 *	@return Returns size of data buffer, in bytes.
	 */
	inline size_t n_Buffer_Size() const
	{
		return m_n_buffer_size;
	}

	/**
	 *	@brief receives requested amount of data
	 *
	 *	@param[out] p_buffer is buffer for incoming data
	 *	@param[in] n_buffer_size is maximal size of data to be written to the buffer
	 *
	 *	@return Returns size of data received, in bytes, or -1 on failure. Can return 0 in case socket is non-blocking.
	 *
	 *	@note In case there are no data for the socket, the function is by default waiting
	 *		  until some arrives (can be overriden by calling SetNonBlocking()).
	 */
	int n_Read(void *p_buffer, size_t n_buffer_size);

	/**
	 *	@brief reads text message, delimited by given sequence of characters
	 *
	 *	@param[out] r_s_output is output string where received message is stored (erased on the beginning)
	 *	@param[in] p_s_delimiter is message delimiter (it will be the rightmost substring of the returned string)
	 *
	 *	@return Returns true on success, false on failure (comm error or not enough memory).
	 *
	 *	@note In case there are no data for the socket, the function is by default waiting
	 *		until some arrives (can *not* be overriden by calling SetNonBlocking(),
	 *		function fails for non-blocking sockets in case there's not enough data).
	 */
	bool ReadMessage(std::string &r_s_output, const char *p_s_delimiter);

	/**
	 *	@brief reads text message
	 *
	 *	Reads text message, while each char is first passed to the accept_char predicate.
	 *	Characters are appended until accept_char returns false. The last character is not part of the message.
	 *
	 *	@param[out] r_s_output is output string where received message is stored (erased on the beginning)
	 *	@param[in] accept_char is function object, (or bool (*function)(char n_char)), returning true in case character should be appended to the message, or false in case the message is complete.
	 *
	 *	@return Returns true on success, false on failure (comm error or not enough memory).
	 */
	template <class CAcceptChar>
	bool ReadMessage2(std::string &r_s_output, CAcceptChar accept_char)
	{
		r_s_output.erase();
		// clean output string

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

			if(!stl_ut::Reserve_NMore(r_s_output, m_p_buffer_end - m_p_buffer_ptr))
				return false;
			// realloc string

			for(;m_p_buffer_ptr != m_p_buffer_end; ++ m_p_buffer_ptr) {
				if(!accept_char(*m_p_buffer_ptr))
					return true;
				// see wheter character is accepted

				r_s_output += *m_p_buffer_ptr;
				// append character
			}
			// append characters until delimiter is contained within the string
		}
	}
};

/**
 *	@brief simple class intended for parsing HTTP response headers
 */
class CHTTP_Header {
protected:
	struct TField {
		std::string s_name, s_value;

		inline bool operator ==(const char *p_s_key) const
		{
			return s_name == p_s_key;
		}
	};
	std::vector<TField> m_field_list;

	bool m_b_response_type;

	std::string m_s_status_line;
	int m_n_status_code;
	// response-type header

	std::string m_s_method, m_s_request_uri;
	// request-type header

public:
	/**
	 *	@brief default constructor; has no 
	 */
	CHTTP_Header();

	/**
	 *	@brief constructor
	 *
	 *	Parses HTTP message p_s_message (which has to be null-terminated), if successful, r_n_message_length
	 *	contains message length (after which body mihgt follow); otherwise it's set to zero (which shouldn't normally occur).
	 *
	 *	@param[in] p_s_message is HTTP header; may be followed by message body
	 *	@param[out] r_n_message_length is number of parsed bytes, message body follows after
	 *
	 *	@note On success, r_n_message_length is nonzero.
	 */
	CHTTP_Header(const char *p_s_message, size_t &r_n_message_length);

	/**
	 *	@brief parses HTTP message p_s_message (which has to be null-terminated)
	 *
	 *	@param[in] p_s_message is HTTP header; may be followed by message body
	 *	@param[out] r_n_message_length is number of parsed bytes, message body follows after
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Parse(const char *p_s_message, size_t &r_n_message_length);

	/**
	 *	@brief determines whether the parsed header was client request or server response
	 *
	 *	@return Returns true in case header contains server response, or false in case header contains client request.
	 *
	 *	@note This is only valid in case message was correctly parsed.
	 */
	inline bool b_ResponseHeader() const
	{
		return m_b_response_type;
	}

	/**
	 *	@brief gets HTTP status code (response-type messages)
	 *
	 *	@return Returns HTTP status code, contained in message status line.
	 *
	 *	@note This is only valid in case message was correctly parsed.
	 */
	inline int n_Status_Code() const
	{
		return m_n_status_code;
	}

	/**
	 *	@brief gets HTTP status line (response-type messages)
	 *
	 *	@return Returns message status line as null terminated string
	 *		(such as "HTTP/1.1 200 OK" for server response headers or "GET / HTTP/1.1" for client request headers).
	 *
	 *	@note This is only valid in case message was correctly parsed.
	 */
	inline const char *p_s_Status_Line() const
	{
		return m_s_status_line.c_str();
	}

	/**
	 *	@brief gets HTTP status line (response-type messages)
	 *
	 *	@return Returns message status line (such as "HTTP/1.1 200 OK" for server
	 *		response headers or "GET / HTTP/1.1" for client request headers).
	 *
	 *	@note This is only valid in case message was correctly parsed.
	 */
	inline const std::string &s_Status_Line() const
	{
		return m_s_status_line;
	}

	/**
	 *	@brief gets HTTP request method
	 *
	 *	@return Returns request method (as "GET", "HEAD" or etc) as null terminated string.
	 *
	 *	@note This is only valid for client request headers
	 */
	inline const char *p_s_Method() const
	{
		return (!m_b_response_type)? m_s_method.c_str() : 0;
	}

	/**
	 *	@brief gets HTTP request method
	 *
	 *	@return Returns request method (as "GET", "HEAD" or etc).
	 *
	 *	@note This is only valid for client request headers
	 */
	inline const std::string &s_Method() const
	{
		return m_s_method;
	}

	/**
	 *	@brief gets HTTP request URI
	 *
	 *	@return Returns request URI as null terminated string.
	 *
	 *	@note This is only valid for client request headers
	 */
	inline const char *p_s_Request_URI() const
	{
		return (!m_b_response_type)? m_s_request_uri.c_str() : 0;
	}

	/**
	 *	@brief gets HTTP request URI
	 *
	 *	@return Returns request URI.
	 *
	 *	@note This is only valid for client request headers
	 */
	inline const std::string &s_Request_URI() const
	{
		return m_s_request_uri;
	}

	/**
	 *	@brief finds HTTP header field
	 *
	 *	@param[in] p_s_name is field name (comparison is case-sensitive)
	 *
	 *	@return Returns value of field named p_s_name as null terminated string,
	 *		in case there is no field with such name, returns 0.
	 */
	const char *p_s_Field(const char *p_s_name) const;

protected:
	bool Add_Field(const char *p_s_name, const char *p_s_name_end, const char *p_s_value);
	bool Parse_Fields(const char *p_s_message, const char *p_s_message_ptr,
		size_t &r_n_message_length);
};

/**
 *	@brief HTTP status code names
 */
enum EHTTPStatusCode {
	http_Continue						= 100, /**< Continue */
	http_SwitchingProtocols				= 101, /**< Switching Protocols */
	http_OK								= 200, /**< OK */
	http_Created						= 201, /**< Created */
	http_Accepted						= 202, /**< Accepted */
	http_Non_AuthoritativeInformation	= 203, /**< Non-Authoritative Information */
	http_NoContent						= 204, /**< No Content */
	http_ResetContent					= 205, /**< Reset Content */
	http_PartialContent					= 206, /**< Partial Content */
	http_MultipleChoices				= 300, /**< Multiple Choices */
	http_MovedPermanently				= 301, /**< Moved Permanently */
	http_Found							= 302, /**< Found */
	http_SeeOther						= 303, /**< See Other */
	http_NotModified					= 304, /**< Not Modified */
	http_UseProxy						= 305, /**< Use Proxy */
	http_TemporaryRedirect				= 307, /**< Temporary Redirect */
	http_BadRequest						= 400, /**< Bad Request */
	http_Unauthorized					= 401, /**< Unauthorized */
	http_PaymentRequired				= 402, /**< Payment Required */
	http_Forbidden						= 403, /**< Forbidden */
	http_NotFound						= 404, /**< Not Found */
	http_MethodNotAllowed				= 405, /**< Method Not Allowed */
	http_NotAcceptable					= 406, /**< Not Acceptable */
	http_ProxyAuthenticationRequired	= 407, /**< Proxy Authentication Required */
	http_RequestTime_out				= 408, /**< Request Time-out */
	http_Conflict						= 409, /**< Conflict */
	http_Gone							= 410, /**< Gone */
	http_LengthRequired					= 411, /**< Length Required */
	http_PreconditionFailed				= 412, /**< Precondition Failed */
	http_RequestEntityTooLarge			= 413, /**< Request Entity Too Large */
	http_Request_URITooLarge			= 414, /**< Request-URI Too Large */
	http_UnsupportedMediaType			= 415, /**< Unsupported Media Type */
	http_RequestedRangeNotSatisfiable	= 416, /**< Requested range not satisfiable */
	http_ExpectationFailed				= 417, /**< Expectation Failed */
	http_InternalServerError			= 500, /**< Internal Server Error */
	http_NotImplemented					= 501, /**< Not Implemented */
	http_BadGateway						= 502, /**< Bad Gateway */
	http_ServiceUnavailable				= 503, /**< Service Unavailable */
	http_GatewayTime_out				= 504, /**< Gateway Time-out */
	http_HTTPVersionnotsupported		= 505  /**< HTTP Version not supported */
};

/**
 *	@brief simple class for receiving files using HTTP protocol
 *
 *	@note Instances of this class do not contain actual file data, they are rather passed to function object.
 */
class CHTTP_File {
protected:
	std::string m_s_location;
	CHTTP_Header *m_p_header;
	uint64_t m_n_size;

public:
	/**
	 *	@brief default constructor; sets file location to p_s_location
	 *
	 *	@param[in] p_s_location is URL in format "http:" "//" host [ ":" port ] [ abs_path [ "?" query ]]
	 *
	 *	@note Location might change after calling Receive() in case file is redirected.
	 */
	CHTTP_File(const char *p_s_location);

	/**
	 *	@brief destructor
	 */
	~CHTTP_File();

	/**
	 *	@brief gets file location (URL)
	 *
	 *	@return returns file location (URL).
	 *
	 *	@note The URL should initially be the same as location, passed to the constructor,
	 *		but might change after calling Receive() in case file is redirected.
	 */
	inline const char *p_s_Location() const
	{
		return m_s_location.c_str();
	}

	/**
	 *	@brief gets file header
	 *	@return Returns pointer to file header, or 0 in case file was not received yet or in case receiving failed.
	 */
	inline const CHTTP_Header *p_Header() const
	{
		return m_p_header;
	}

	/**
	 *	@brief gets file size
	 *
	 *	@return Returns file size, or 0 in case file was not received yet, in case receiving failed,
	 *		in case server replied with no-content message or in case transfer encoding is chunked
	 *		and file size is not known in advance.
	 */
	inline uint64_t n_Size() const
	{
		return m_n_size;
	}

	/**
	 *	@brief downloads the file
	 *
	 *	@param[out] pass_data is function object with "bool operator ()(const void *p_data, int n_size)" file is passed to
	 *	@param[in] p_s_request_params is string with HTTP request parameters (such as "User-Agent: Mozilla/4.0\r\n" or
	 *		"Content-Length: 11\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\nkey=12345\r\n")
	 *	@param[in] n_allow_redirect_num is maximal number of redirections (HTTP 300)
	 *	@param[in] n_buffer_size is HTTP socket buffer size
	 *	@param[in] b_use_post decides between HTTP GET (false) or HTTP POST (true) request
	 *	@param[in] n_timeout_seconds is timeout in seconds; setting it it -1 leaves defaut CHTTP_Socket
	 *		timeout, setting it to 0 disables timeout (sets infinite timeout)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CDataTarget>
	inline bool Receive(CDataTarget pass_data, const char *p_s_request_params = 0,
		int n_allow_redirect_num = 5, size_t n_buffer_size = 4096,
		bool b_use_post = false, int n_timeout_seconds = -1)
	{
		return ProxiedReceive(0, pass_data, p_s_request_params, n_allow_redirect_num,
			n_buffer_size, b_use_post, n_timeout_seconds);
	}

	/**
	 *	@brief downloads the file via HTTP proxy
	 *
	 *	@param[in] p_s_proxy is URL of HTTP proxy server
	 *	@param[out] pass_data is function object with "bool operator ()(const void *p_data, int n_size)" file is passed to
	 *	@param[in] p_s_request_params is string with HTTP request parameters (such as "User-Agent: Mozilla/4.0\r\n" or
	 *		"Content-Length: 11\r\nContent-Type: application/x-www-form-urlencoded\r\n\r\nkey=12345\r\n")
	 *	@param[in] n_allow_redirect_num is maximal number of redirections (HTTP 300)
	 *	@param[in] n_buffer_size is HTTP socket buffer size
	 *	@param[in] b_use_post decides between HTTP GET (false) or HTTP POST (true) request
	 *	@param[in] n_timeout_seconds is timeout in seconds; setting it it -1 leaves defaut CHTTP_Socket
	 *		timeout, setting it to 0 disables timeout (sets infinite timeout)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CDataTarget>
	bool ProxiedReceive(const char *p_s_proxy,
		CDataTarget pass_data, const char *p_s_request_params = 0,
		int n_allow_redirect_num = 5, size_t n_buffer_size = 4096,
		bool b_use_post = false, int n_timeout_seconds = -1)
	{
		for(int n_redirect_num = 0;; ++ n_redirect_num) {
			if(m_p_header) {
				delete m_p_header;
				m_p_header = 0;
			}
			// delete header if present

			m_n_size = 0;
			// reset file-size

			CHTTP_Socket t_socket(n_buffer_size);
			if(n_timeout_seconds >= 0)
				t_socket.SetTimeout(n_timeout_seconds);
			if(p_s_proxy) {
				if(!Connect_SendGET(p_s_proxy, t_socket, b_use_post, p_s_request_params))
					return false;
			} else {
				if(!Connect_SendGET(t_socket, b_use_post, p_s_request_params))
					return false;
			}
			// connect to host and send GET request

			for(int n_continue_num = 0;; ++ n_continue_num) {
				std::string s_message;
				if(!t_socket.ReadMessage(s_message, "\r\n\r\n"))
					return false;
				// read response header (terminated by double newline)

				size_t n_response_parsed;
				if(!(m_p_header = new(std::nothrow) CHTTP_Header(s_message.c_str(),
				   n_response_parsed)) || !n_response_parsed)
					return false;
				_ASSERTE(n_response_parsed == s_message.length());
				if(!m_p_header->b_ResponseHeader())
					return false; // guess it couldn't possibly happen
				// read and parse response

				if(m_p_header->n_Status_Code() == http_Continue) {
					if(n_continue_num > 50)
						return false;
					delete m_p_header;
				} else
					break;
			}
			// read header, skip "Continue" headers

			if(m_p_header->n_Status_Code() == http_NoContent ||
			   m_p_header->n_Status_Code() == http_NotModified) {
				_ASSERTE(m_p_header->n_Status_Code() != http_NotModified);
				// shouldn't occur; response to conditional requests

				m_n_size = 0; // no-content marks that no body follows
				return true;
			} else if(m_p_header->n_Status_Code() >= 300 && m_p_header->n_Status_Code() < 400) {
				const char *p_s_location;
				if(!(p_s_location = m_p_header->p_s_Field("Location")))
					return false;
				// header field location should contain redirection URL

				__STL_UT_TRY {
					if(p_s_location[0] != '/') {
						m_s_location = p_s_location;
					} else {
						std::string s_hostname;
						s_hostname = m_s_location;
						if(s_hostname.find("://") != std::string::npos) {
							s_hostname.erase(s_hostname.begin(), s_hostname.begin() +
								(s_hostname.find("://") + 3));
						}
						if(s_hostname.find("/") != std::string::npos) {
							s_hostname.erase(s_hostname.begin() +
								s_hostname.find('/'), s_hostname.end());
						}
						// make hostname string

						m_s_location = s_hostname;
						m_s_location += p_s_location;
					}
					// concat new location with hostname
				} __STL_UT_CATCH_BAD_ALLOC {
					return false;
					// memory allocation error
				}

				if(n_redirect_num >= n_allow_redirect_num)
					return false;
				// we're over-redirected

				continue;
				// try with a new location
			} else if(m_p_header->n_Status_Code() >= 200 && m_p_header->n_Status_Code() < 300) {
				// we should have our file

				return Receive_Data(pass_data, m_p_header, &t_socket, m_n_size);
			} else
				return false;
			// see what response code is

			break;
		}
		// try receiving file several times (might get redirected)

		return true;
	}

	/**
	 *	@brief receives file data
	 *
	 *	@param[out] pass_data is function object with "bool operator ()(const void *p_data, int n_size)" file is passed to
	 *	@param[in] p_header is HTTP header (should contain "200 OK"-like response)
	 *	@param[in] p_socket is socket, connected to HTTP server
	 *	@param[out] r_n_file_size is file size counter
	 *
	 *	@return Returns true on success, false on failure.
	 */
	template <class CDataTarget>
	static bool Receive_Data(CDataTarget pass_data, CHTTP_Header *p_header,
		CHTTP_Socket *p_socket, uint64_t &r_n_file_size)
	{
		if(p_header->p_s_Field("Transfer-Encoding") &&
		   !strcmp(p_header->p_s_Field("Transfer-Encoding"), "chunked")) {
			// chunked-coded file

			uint8_t *p_chunk;
			if(!(p_chunk = new(std::nothrow) uint8_t[p_socket->n_Buffer_Size()]))
				return false;
			// pre-allocate chunk buffer

			r_n_file_size = 0;
			for(;;) {
				std::string s_chunk_length;
				if(!p_socket->ReadMessage2(s_chunk_length, b_IsHexaDigit))
					return false;
				size_t n_chunk_length;
				//int n_chunk_length32;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
				sscanf_s(s_chunk_length.c_str(), PRIsizex, &n_chunk_length);
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				sscanf(s_chunk_length.c_str(), PRIsizex, &n_chunk_length);
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
				//n_chunk_length = n_chunk_length32;
				// get chunk length, 

				if(!n_chunk_length)
					break;
				// zero chunk length terminates transfer

				r_n_file_size += n_chunk_length;

				if(!p_socket->ReadMessage(s_chunk_length, "\r\n")) {
					delete[] p_chunk;
					return false;
				}
				// skip to the end of line

				for(size_t n_read = n_chunk_length; n_read;) {
					size_t n_dose = (p_socket->n_Buffered())?
						p_socket->n_Buffered() : p_socket->n_Buffer_Size();
					// eat up buffered data first to minimize number of buffer re-fills
					if(n_dose > n_read)
						n_dose = n_read;
					_ASSERTE(n_dose <= p_socket->n_Buffer_Size());
					if(p_socket->n_Read(p_chunk, n_dose) != (signed)n_dose ||
					   !pass_data((const void*)p_chunk, n_dose)) {
						delete[] p_chunk;
						return false;
					}
					n_read -= n_dose;
				}
				// read chunk in p_socket->n_Buffer_Size() chunks

				char p_s_crlf[2];
				if(p_socket->n_Read(p_s_crlf, 2 * sizeof(char)) != 2 ||
				   p_s_crlf[0] != '\r' || p_s_crlf[1] != '\n') {
					delete[] p_chunk;
					return false;
				}
				// cr-lf follows
			}
			delete[] p_chunk;
			// delete chunk buffer

			std::string s_trailer;
			if(!p_socket->ReadMessage(s_trailer, "\r\n\r\n"))
				return false;
			// read message trailer

			size_t n_trailer_parsed;
			if(!p_header->Parse(s_trailer.c_str(), n_trailer_parsed) ||
			   !n_trailer_parsed /*!= s_trailer.length()*/)
				return false;
			// parse trailer
		} else if(p_header->p_s_Field("Content-Length")) {
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
			if(sscanf_s(p_header->p_s_Field("Content-Length"), PRIu64, &r_n_file_size) != 1) // good for 64-bit
				return false;
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			{
				//if(sscanf(p_header->p_s_Field("Content-Length"), PRIu64, &r_n_file_size) != 1) // some problems on pcivs
				const char *p_s_cl = p_header->p_s_Field("Content-Length");
				while(p_s_cl && *p_s_cl && isspace(uint8_t(*p_s_cl)))
					++ p_s_cl; // skip space
				if(!p_s_cl || !*p_s_cl || !isdigit(uint8_t(*p_s_cl)))
					return false; // is it a number
				r_n_file_size = strtoul(p_header->p_s_Field("Content-Length"), NULL, 10);
			}
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			// simple message with given size

			uint8_t *p_chunk;
			if(!(p_chunk = new(std::nothrow) uint8_t[p_socket->n_Buffer_Size()]))
				return false;
			// pre-allocate chunk buffer

			for(uint64_t n_read = r_n_file_size; n_read;) {
				size_t n_dose = (p_socket->n_Buffered())?
					p_socket->n_Buffered() : p_socket->n_Buffer_Size();
				// eat up buffered data first to minimize number of buffer re-fills
				if(n_dose > n_read) {
					_ASSERTE(n_read <= SIZE_MAX);
					n_dose = size_t(n_read);
				}
				_ASSERTE(n_dose <= p_socket->n_Buffer_Size());
				if(p_socket->n_Read(p_chunk, n_dose) != (signed)n_dose ||
				   !pass_data((const void*)p_chunk, n_dose)) {
					delete[] p_chunk;
					return false;
				}
				n_read -= n_dose;
			}
			delete[] p_chunk;
			// receive file in p_socket->n_Buffer_Size() chunks (progress-bar purposes)
		} else if(p_header->p_s_Field("Connection") &&
		   !strcmp(p_header->p_s_Field("Connection"), "close")) {
			r_n_file_size = 0; // terminated by closing connection

			uint8_t *p_chunk;
			if(!(p_chunk = new(std::nothrow) uint8_t[p_socket->n_Buffer_Size()]))
				return false;
			// pre-allocate chunk buffer

			for(;;) {
				size_t n_dose = (p_socket->n_Buffered())?
					p_socket->n_Buffered() : p_socket->n_Buffer_Size();
				// eat up buffered data first to minimize number of buffer re-fills

				_ASSERTE(n_dose <= p_socket->n_Buffer_Size());
				int n_read = p_socket->n_Read(p_chunk, n_dose);
				if(!n_read)
					break;
				if(n_read < 0 || !pass_data((const void*)p_chunk, n_dose)) {
					delete[] p_chunk;
					return false;
				}
				r_n_file_size += n_read;
			}
			delete[] p_chunk;
			// receive file in p_socket->n_Buffer_Size() chunks (progress-bar purposes)

			return true;
		} else {
			r_n_file_size = 0;
			return false; // unable to tell size
		}

		return true;
	}

protected:
	static inline bool b_IsHexaDigit(char n_char)
	{
		return (n_char >= '0' && n_char <= '9') || ((n_char | 32) >= 'a' && (n_char | 32) <= 'z');
	}
	bool Connect_SendGET(CHTTP_Socket &r_socket, bool b_use_post, const char *p_s_request_params = 0);
	bool Connect_SendGET(const char *p_s_proxy, CHTTP_Socket &r_socket, bool b_use_post, const char *p_s_request_params = 0);
};

#endif // !__HTTP_CLIENT_INCLUDED
