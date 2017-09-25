/*
								+---------------------------------+
								|                                 |
								|  ***   Simple HTTP server  ***  |
								|                                 |
								|  Copyright  © -tHE SWINe- 2007  |
								|                                 |
								|         HTTP_Server.cpp         |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file proto/HTTP_Server.cpp
 *	@brief a simple HTTP server base class
 *	@author -tHE SWINe-
 *	@date 2007
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include <string>
#include <vector>
#include <algorithm>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include "../Socket.h"
#include "../Thread.h"
#include "HTTP_Client.h"
#include "HTTP_Server.h"

#if !defined(_WIN32) && !defined(_WIN64)
#include <time.h>
#include <unistd.h> // sleep()
static void Sleep(int n_millis)
{
	struct timespec t_time;
	memset(&t_time, 0, sizeof(t_time));
	t_time.tv_sec = n_millis / 1000;
	t_time.tv_nsec = (n_millis % 1000) * 1000000;
	nanosleep(&t_time, NULL);
}
#endif // !_WIN32 && !_WIN64

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/*
 *								=== CHTTP_Server::CWorkingThread ===
 */

CHTTP_Server::CWorkingThread::CWorkingThread(CHTTP_Server *p_server, bool &r_b_running,
	CHTTP_Socket *p_listening_socket, CMutex *p_listening_socket_mutex)
	:m_p_server(p_server), m_p_listening_socket(p_listening_socket),
	m_p_listening_socket_mutex(p_listening_socket_mutex), m_r_b_running(r_b_running)
{
	m_thread.AttachRunable(*this);
}

bool CHTTP_Server::CWorkingThread::WaitForAccept()
{
	return m_thread.Start(); // start this thread
}

bool CHTTP_Server::CWorkingThread::WaitUntilFinished()
{
	return m_thread.Stop(false);
}

void CHTTP_Server::CWorkingThread::Run()
{
	for(;;) {
		if(!m_p_listening_socket_mutex->Lock()) {
			fprintf(stderr, "fatal error: failed to enter critical section\n");
			return;
		}
		// lock mutex

		if(!m_r_b_running) {
			if(!m_p_listening_socket_mutex->Unlock()) {
				fprintf(stderr, "fatal error: failed to leave critical section while quitting\n");
				return;
			}
			return;
		}
		// if the server is no longer running, unlock the mutex for the other workers to quit, and exit

		TNetAddress t_client_address;
		CHTTP_Socket *p_socket;
		if(!(p_socket = m_p_listening_socket->p_Accept(t_client_address, 4096, 600))) { // 10 minutes timeout
			if(!m_p_listening_socket_mutex->Unlock()) {
				fprintf(stderr, "fatal error: failed to leave critical section\n");
				return;
			}
			continue;
		}
		// try to accept incoming connections

		int n_tid = CCurrentThreadHandle::n_Get_Id();
		// get thread identifier

		m_p_listening_socket_mutex->Unlock();
		// unlock mutex, let other threads accept connections

		if(!m_r_b_running) {
			delete p_socket;
			return;
		}
		// in case the server is quitting and this worker was woken
		// up by the dummy connection, close the socket and quit

		m_p_server->Handle_Request(p_socket, n_tid);
		// handle request; what to do in case errors occur?

		delete p_socket;
		// close connection
	}
}

/*
 *								=== ~CHTTP_Server::CWorkingThread ===
 */

/*
 *								=== CHTTP_Server ===
 */

bool CHTTP_Server::Listen(int n_port, int n_max_conns, bool b_allow_threaded)
{
	if(m_b_running)
		return m_n_port == n_port; // another job well done ... maybe
	if(n_max_conns < 1)
		n_max_conns = 1;

	if(!m_listening_socket.Listen(TNetAddress(0, n_port), n_max_conns))
		return false;
	// create listen socket

	m_b_running = true;

	if(b_allow_threaded) {
		std::vector<CWorkingThread*> m_worker_list;
		if(!stl_ut::Resize_To_N(m_worker_list, n_max_conns))
			return false;
		for(int i = 0; i < n_max_conns; ++ i) {
			if(!(m_worker_list[i] = new(std::nothrow) CWorkingThread(this,
			   m_b_running, &m_listening_socket, &m_listen_mutex))) {
				for(-- i; i = 0; -- i)
					delete m_worker_list[i];
				m_worker_list.clear();
				return false;
			}
		}
		// create workers

		for(int i = 0; i < n_max_conns; ++ i)
			m_worker_list[i]->WaitForAccept();
		// start them

		for(int i = 0; i < n_max_conns; ++ i) {
			m_worker_list[i]->WaitUntilFinished();
			delete m_worker_list[i];
		}
		// clean-up (never)

		m_worker_list.clear();
	} else {
		for(;;) {
			TNetAddress t_client_address;
			CHTTP_Socket *p_socket;
			if(!(p_socket = m_listening_socket.p_Accept(t_client_address, 4096, 600))) // 10 minute timeout
				continue;
			// try to accept incoming connections

			Handle_Request(p_socket, 0);
			// handle request; what to do in case errors occur?

			delete p_socket;
			// close connection
		}
	}

	m_b_running = false; // never happens (unreachable code)

	return true;
}

bool CHTTP_Server::Listen_Start(int n_port, int n_max_conns)
{
	if(m_b_running)
		return m_n_port == n_port; // another job well done ... maybe
	if(n_max_conns < 1)
		n_max_conns = 1;

	if(!m_listening_socket.Listen(TNetAddress(0, n_port), n_max_conns))
		return false;
	// create listen socket

	m_b_running = true;
	m_n_port = n_port;

	if(!stl_ut::Resize_To_N(m_worker_list, n_max_conns))
		return false;
	for(int i = 0; i < n_max_conns; ++ i) {
		if(!(m_worker_list[i] = new(std::nothrow) CWorkingThread(this,
		   m_b_running, &m_listening_socket, &m_listen_mutex))) {
			for(-- i; i = 0; -- i)
				delete m_worker_list[i];
			m_worker_list.clear();
			m_b_running = false;
			return false;
		}
	}
	// create workers

	m_b_running = true;
	m_n_port = n_port;

	for(int i = 0; i < n_max_conns; ++ i)
		m_worker_list[i]->WaitForAccept();
	// start them

	return true;
}

bool CHTTP_Server::Listen_End()
{
	if(!m_b_running)
		return true; // another job well done
	m_b_running = false;
	// clear running flag

	for(int n_fail_num = 0; n_fail_num < 1000; ++ n_fail_num) {
		CStreamSocket sock;
		if(sock.Connect(TNetAddress("127.0.0.1", m_n_port))) // f_ixme - does this work? // seems to, although it is probably not the best solution
			sock.Close();
		// try to connect to one of the workers (to wake it up)

		for(size_t i = 0, n = m_worker_list.size(); i < n; ++ i) {
			if(!m_worker_list[i]->b_Running()) {
				m_worker_list[i]->WaitUntilFinished(); // no-op
				delete m_worker_list[i];
				m_worker_list.erase(m_worker_list.begin() + i);
				-- i;
				-- n;
			}
			// in case any of the workers stopped, remove it from the list
		}

		if(m_worker_list.empty())
			return true;

		Sleep(100);
		// maybe someone is being served; wait
	}
	// try stopping the server several times

	return false;
}

bool CHTTP_Server::Handle_Request(CHTTP_Socket *p_socket, int n_tid)
{
	std::string s_message;
	if(!p_socket->ReadMessage(s_message, "\r\n\r\n"))
		return false;
	// read response header (terminated by double newline)

	size_t n_response_parsed;
	CHTTP_Header *p_header;
	//printf("%s<<\n", s_message.c_str());
	if(!(p_header = new(std::nothrow) CHTTP_Header(s_message.c_str(),
	   n_response_parsed)) || !n_response_parsed)
		return false;
	_ASSERTE(n_response_parsed == s_message.length());
	// read and parse response

	bool b_result;
	if(p_header->b_ResponseHeader() || !p_header->p_s_Method())
		b_result = Handle_BadRequest(p_header, p_socket);
	else if(!strcmp(p_header->p_s_Method(), "GET"))
		b_result = Handle_GET(p_header, p_socket, n_tid);
	else if(!strcmp(p_header->p_s_Method(), "POST"))
		b_result = Handle_POST(p_header, p_socket, n_tid);
	else if(!strcmp(p_header->p_s_Method(), "HEAD"))
		b_result = Handle_HEAD(p_header, p_socket, n_tid);
	else if(!strcmp(p_header->p_s_Method(), "PUT"))
		b_result = Handle_PUT(p_header, p_socket, n_tid);
	else if(!strcmp(p_header->p_s_Method(), "DELETE") ||
			!strcmp(p_header->p_s_Method(), "OPTIONS") ||
			!strcmp(p_header->p_s_Method(), "TRACE")) {
		b_result = Handle_NotImplemented(p_header, p_socket);
		// unimplemented methods (extension possible by rewriting Handle_NotImplemented())
	} else
		b_result = Handle_BadRequest(p_header, p_socket); // unknown method
	// call appropriate handler

	delete p_header;
	// delete header

	return b_result;
}

bool CHTTP_Server::Handle_BadRequest(const CHTTP_Header *p_header,
	CHTTP_Socket *p_socket, const char *p_s_content) const
{
	int n_code = 400;
	const char *p_s_message = "Bad Request";
	//const char *p_s_content = "<html><body><h2>HTTP/1.1 400</h2><div>Bad Request</div></body></html>";

	std::string s_response, s_date;
	return stl_ut::Format(s_response, "HTTP/1.1 %d %s\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Content-Type: text/html\r\n" // browser would't display text message when retreiving image
		"Content-Length: %d\r\n\r\n"
		"%s", n_code, p_s_message, p_s_DateString(s_date), strlen(p_s_content), p_s_content) &&
		p_socket->n_Write(s_response.data(), s_response.length()) == s_response.length();
	// reply with "bad request" and that's it
}

bool CHTTP_Server::Handle_NotFound(const CHTTP_Header *p_header,
	CHTTP_Socket *p_socket, const char *p_s_content) const
{
	int n_code = 404;
	const char *p_s_message = "Not Found";
	//const char *p_s_content = "<html><body><h2>HTTP/1.1 404</h2><div>Not Found</div></body></html>";

	std::string s_response, s_date;
	return stl_ut::Format(s_response, "HTTP/1.1 %d %s\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Content-Type: text/html\r\n" // browser would't display text message when retreiving image
		"Content-Length: %d\r\n\r\n"
		"%s", n_code, p_s_message, p_s_DateString(s_date), strlen(p_s_content), p_s_content) &&
		p_socket->n_Write(s_response.data(), s_response.length()) == s_response.length();
	// reply with "bad request" and that's it
}

bool CHTTP_Server::Handle_InternalServerError(const CHTTP_Header *p_header,
	CHTTP_Socket *p_socket, const char *p_s_content) const
{
	int n_code = 500;
	const char *p_s_message = "Internal Server Error";
	//const char *p_s_content = "<html><body><h2>HTTP/1.1 500</h2><div>Internal Server Error</div></body></html>";

	std::string s_response, s_date;
	return stl_ut::Format(s_response, "HTTP/1.1 %d %s\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Content-Type: text/html\r\n" // browser would't display text message when retreiving image
		"Content-Length: %d\r\n\r\n"
		"%s", n_code, p_s_message, p_s_DateString(s_date), strlen(p_s_content), p_s_content) &&
		p_socket->n_Write(s_response.data(), s_response.length()) == s_response.length();
	// reply with "bad request" and that's it
}

bool CHTTP_Server::Handle_NotImplemented(const CHTTP_Header *p_header,
	CHTTP_Socket *p_socket, const char *p_s_content) const
{	
	int n_code = 501;
	const char *p_s_message = "Not Implemented";
	//const char *p_s_content = "<html><body><h2>HTTP/1.1 501</h2><div>Not Implemented</div></body></html>";

	std::string s_response, s_date;
	return stl_ut::Format(s_response, "HTTP/1.1 %d %s\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Content-Type: text/html\r\n" // browser would't display text message when retreiving image
		"Content-Length: %d\r\n\r\n"
		"%s", n_code, p_s_message, p_s_DateString(s_date), strlen(p_s_content), p_s_content) &&
		p_socket->n_Write(s_response.data(), s_response.length()) == s_response.length();
	// reply with "bad request" and that's it
}

bool CHTTP_Server::Handle_HEAD(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid)
{
	return Handle_GET(p_header, p_socket, n_tid);
}

bool CHTTP_Server::Handle_POST(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid)
{
	return Handle_GET(p_header, p_socket, n_tid);
}

bool CHTTP_Server::Handle_GET(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid)
{
	_ASSERTE(p_header && p_header->p_s_Request_URI());
	// there must be request URI

	if(!strstr(p_header->p_s_Status_Line(), "HTTP/1.0") &&
	   !strstr(p_header->p_s_Status_Line(), "HTTP/1.1") && !p_header->p_s_Field("Host"))
		return Handle_BadRequest(p_header, p_socket);
	// header must contain host field (aparts from HTTP/1.0 requests)

	bool b_method_get = strcmp(p_header->p_s_Method(), "HEAD") != 0;
	_ASSERTE(!b_method_get || !strcmp(p_header->p_s_Method(), "GET") ||
		!strcmp(p_header->p_s_Method(), "POST"));
	// determine wheter GET/POST or HEAD

	std::string s_file;
	if(!CURI_Utils::Get_FileName_Part(s_file, p_header->s_Request_URI()))
		return Handle_InternalServerError(p_header, p_socket);
	const char *p_s_file = s_file.c_str();
	// extract desired filename along with path; handle absolute URI's

	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_file, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_file, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return Handle_NotFound(p_header, p_socket);
	if(fseek(p_fr, 0, SEEK_END)) {
		fclose(p_fr);
		return Handle_InternalServerError(p_header, p_socket);
	}
	long n_file_size = ftell(p_fr);
	unsigned char *p_file_buffer;
	if(b_method_get) {
		if(!(p_file_buffer = new(std::nothrow) unsigned char[n_file_size])) {
			fclose(p_fr);
			return Handle_InternalServerError(p_header, p_socket);
		}
		if(fseek(p_fr, 0, SEEK_SET) || fread(p_file_buffer, n_file_size, 1, p_fr) != 1) {
			delete[] p_file_buffer;
			fclose(p_fr);
			return Handle_InternalServerError(p_header, p_socket);
		}
	}
	fclose(p_fr);
	// determine file size, might return "404: not found" or "500: server error" on io error

	std::string s_response, s_date;
	if(!stl_ut::Format(s_response, "HTTP/1.1 200 OK\r\n"
	   "Date: %s\r\n"
	   "Connection: close\r\n"
	   "Content-Length: %ld\r\n\r\n", p_s_DateString(s_date), n_file_size))
		return Handle_InternalServerError(p_header, p_socket);
	// create response string (contains no html body)

	for(int n_pass = 0; n_pass < ((b_method_get)? 2 : 1); ++ n_pass) {
		unsigned int n_to_write = (n_pass)? n_file_size : s_response.length();
		const unsigned char *p_write_ptr = (n_pass)? p_file_buffer :
			(const unsigned char*)s_response.data();
		while(n_to_write) {
			int n_written;
			if((n_written = p_socket->n_Write(p_write_ptr, n_to_write)) <= 0) {
				if(b_method_get)
					delete[] p_file_buffer;
				return false;
			}
			n_to_write -= n_written;
			p_write_ptr += n_written;
		}
	}
	// send header + data

	if(b_method_get)
		delete[] p_file_buffer;
	// cleanup

	return true;
}

class CDataWriter {
protected:
	FILE *m_p_fw;

public:
	inline CDataWriter(FILE *p_fw)
		:m_p_fw(p_fw)
	{}

	inline bool operator ()(const void *p_data, unsigned int n_size)
	{
		return fwrite(p_data, n_size * sizeof(char), 1, m_p_fw) == 1;
	}
};

bool CHTTP_Server::Handle_PUT(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid)
{
	if(!strstr(p_header->p_s_Status_Line(), "HTTP/1.0") &&
	   !strstr(p_header->p_s_Status_Line(), "HTTP/1.1") && !p_header->p_s_Field("Host"))
		return Handle_BadRequest(p_header, p_socket);
	// header must contain host field (aparts from HTTP/1.0 requests)

	std::string s_file;
	if(!CURI_Utils::Get_FileName_Part(s_file, p_header->s_Request_URI()))
		return Handle_InternalServerError(p_header, p_socket);
	const char *p_s_file = s_file.c_str();
	// extract desired filename along with path; handle absolute URI's

	FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_file, "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_file, "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return Handle_InternalServerError(p_header, p_socket);
	// open file

	std::string s_response, s_date;
	if(!stl_ut::Format(s_response, "HTTP/1.1 200 OK\r\n"
	   "Date: %s\r\n\r\n", p_s_DateString(s_date)))
		return Handle_InternalServerError(p_header, p_socket);
	// create response string (contains no html body)

	if(p_socket->n_Write(s_response.data(), s_response.length()) != s_response.length())
		return false;
	// respond "200 OK" (maybe use "201 Created" as well)

	uint64_t n_file_size;
	CHTTP_Header header_copy(*p_header);
	if(!CHTTP_File::Receive_Data(CDataWriter(p_fw), &header_copy, p_socket, n_file_size))
		return false;
	// receive data

	fclose(p_fw);
	// close file

	return true;
}

const char *CHTTP_Server::p_s_DateString(std::string &r_s_dest)
{
	time_t t_time = time(NULL);
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	struct tm timestorage;
	const struct tm *timeptr = &timestorage;
	gmtime_s(&timestorage, &t_time);
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	const struct tm *timeptr = gmtime(&t_time);
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	const char *p_day_name_list[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	const char *p_mon_name_list[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
									 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

	if(!stl_ut::Format(r_s_dest, "%3s, %02d %3s %04d %02d:%02d:%02d GMT",
	   p_day_name_list[timeptr->tm_wday], timeptr->tm_mday, p_mon_name_list[timeptr->tm_mon],
	   1900 + timeptr->tm_year, timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec))
		return "";

	return r_s_dest.c_str();
}

/*
 *								=== ~CHTTP_Server ===
 */
