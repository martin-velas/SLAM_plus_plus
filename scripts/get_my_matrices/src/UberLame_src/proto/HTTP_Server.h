/*
								+---------------------------------+
								|                                 |
								|  ***   Simple HTTP server  ***  |
								|                                 |
								|  Copyright  © -tHE SWINe- 2007  |
								|                                 |
								|          HTTP_Server.h          |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __HTTP_SERVER_INCLUDED
#define __HTTP_SERVER_INCLUDED

/**
 *	@file proto/HTTP_Server.h
 *	@brief a simple HTTP server base class
 *	@author -tHE SWINe-
 *	@date 2007
 *
 *	@date 2012-04-16
 *
 *	Added the CHTTP_Server::Listen_Start() and CHTTP_Server::Listen_End() functions,
 *	providing multithreaded server functionality without wasting one thread to listen
 *	forever and with stopping capability.
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 *	Unified linux / windows line ends.
 *
 */

#include "../Thread.h"
#include "HTTP_Client.h"
#include "URI.h"

/**
 *	@brief minimal HTTP server, handling only very basic requests (deigned for easy extension / customization)
 *
 *	@note It can be a little bit more pedantic than HTTP specs desire. Can handle GET, HEAD and PUT requests.
 */
class CHTTP_Server {
protected:
	class CWorkingThread : public CRunable {
	protected:
		CHTTP_Server *m_p_server;
		CHTTP_Socket *m_p_listening_socket;
		CMutex *m_p_listening_socket_mutex;
		CThread m_thread;
		bool &m_r_b_running;

	public:
		CWorkingThread(CHTTP_Server *p_server, bool &r_b_running,
			CHTTP_Socket *p_listening_socket, CMutex *p_listening_socket_mutex);

		bool WaitForAccept();
		bool WaitUntilFinished();

		inline bool b_Running() const
		{
			return m_thread.b_IsRunning();
		}

	protected:
		virtual void Run();
	};
	friend class CWorkingThread; // g++ requires "class"

	CHTTP_Socket m_listening_socket;
	CMutex m_listen_mutex;
	std::vector<CWorkingThread*> m_worker_list;
	bool m_b_running;
	int m_n_port;

public:
	inline CHTTP_Server()
		:m_listening_socket(0) /* no buffer needed for this socket */,
		m_b_running(false), m_n_port(0)
	{}

	/**
	 *	@brief starts listening on port n_port while accepting up to n_max_conns simultaneous connections
	 *
	 *	@param[in] n_port is port to listen on
	 *	@param[in] n_max_conns is maximal number of incoming connections (socket backlog and number of threads)
	 *	@param[in] b_allow_threaded is multithreaded processing flag, setting
	 *		it to true enables parallel processing of client requests.
	 *
	 *	@return Returns false on failure.
	 *
	 *	@note In original (not overloaded) version this never returns true (runs forever).
	 */
	bool Listen(int n_port, int n_max_conns, bool b_allow_threaded = true);

	/**
	 *	@brief starts listening on port n_port while accepting up to n_max_conns
	 *		simultaneous connections and returns immediately
	 *
	 *	In case the server is already running and the port matches n_port,
	 *	returns true and nothing is changed. In case the server is running on a
	 *	different port, returns false (but keeps the server running).
	 *
	 *	Use Listen_End() to shut down the server.
	 *
	 *	@param[in] n_port is port to listen on
	 *	@param[in] n_max_conns is maximal number of incoming connections (socket backlog and number of threads)
	 *	@param[in] b_allow_threaded is multithreaded processing flag, setting
	 *		it to true enables parallel processing of client requests.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This is a non-blocking version of Listen().
	 */
	bool Listen_Start(int n_port, int n_max_conns);

	/**
	 *	@brief stops listening on port given by Listen_Start()
	 *
	 *	In case the server is not running, does nothing and returns true.
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This may take a long time if there is only a single thread (n_max_conns was <= 1)
	 *		and someone is connected (the connection needs to be stopped in server
	 *		routines first - use the m_b_running flag).
	 */
	bool Listen_End();

private:
	/**
	 *	@brief handles client request
	 *
	 *	Client has connected to p_socket; reads request header and passes
	 *		it to proper virtual handler function (they're supposed to be custom rewritten).
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Handle_Request(CHTTP_Socket *p_socket, int n_tid);

protected:
	/**
	 *	@brief overloadable "400 - bad request" error handler
	 *
	 *	Those handlers are supposed to send client error response and maybe generate some event on server-side (write log, etc).
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] p_s_content is HTTP message content
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_BadRequest(const CHTTP_Header *p_header, CHTTP_Socket *p_socket,
		const char *p_s_content = "<html><body><h2>HTTP/1.1 400</h2><div>Bad Request</div></body></html>") const;

	/**
	 *	@brief overloadable "404 - file not found" error handler
	 *
	 *	Those handlers are supposed to send client error response and maybe generate some event on server-side (write log, etc).
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] p_s_content is HTTP message content
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_NotFound(const CHTTP_Header *p_header, CHTTP_Socket *p_socket,
		const char *p_s_content = "<html><body><h2>HTTP/1.1 404</h2><div>Not Found</div></body></html>") const;

	/**
	 *	@brief overloadable "500 - internal server error" error handler
	 *
	 *	Those handlers are supposed to send client error response and maybe generate some event on server-side (write log, etc).
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] p_s_content is HTTP message content
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_InternalServerError(const CHTTP_Header *p_header, CHTTP_Socket *p_socket,
		const char *p_s_content = "<html><body><h2>HTTP/1.1 500</h2><div>Internal Server Error</div></body></html>") const;

	/**
	 *	@brief overloadable "501 - not implemented" error handler
	 *
	 *	Those handlers are supposed to send client error response and maybe generate some event on server-side (write log, etc).
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] p_s_content is HTTP message content
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_NotImplemented(const CHTTP_Header *p_header, CHTTP_Socket *p_socket,
		const char *p_s_content = "<html><body><h2>HTTP/1.1 501</h2><div>Not Implemented</div></body></html>") const;

	/**
	 *	@brief overloadable "HTTP GET" request handler
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] n_tid is processing thread id (for logging purposes rather than practical thread identification)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_GET(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid);

	/**
	 *	@brief overloadable "HTTP HEAD" request handler
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] n_tid is processing thread id (for logging purposes rather than practical thread identification)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_HEAD(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid);

	/**
	 *	@brief overloadable "HTTP POST" request handler
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] n_tid is processing thread id (for logging purposes rather than practical thread identification)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_POST(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid);

	/**
	 *	@brief overloadable "HTTP PUT" request handler
	 *
	 *	@param[in] p_header is pointer to request header
	 *	@param[in] p_socket is socket with client on the other side
	 *	@param[in] n_tid is processing thread id (for logging purposes rather than practical thread identification)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note 'failure' here refers to error in processing client's request. ie. in case
	 *		it's possible to complete client's request, even trough error response (such as 404), it's not failure.
	 */
	virtual bool Handle_PUT(const CHTTP_Header *p_header, CHTTP_Socket *p_socket, int n_tid);

	/**
	 *	@brief reentrant utility function to get current date (for logging purposes / etc)
	 *
	 *	@param[out] r_s_dest is buffer where the time string is stored
	 *
	 *	@return Returns current time in format "Date: day, dd mon yyyy hh:mm:ss GMT" (without any endlines).
	 */
	static const char *p_s_DateString(std::string &r_s_dest);
};

#endif // !__HTTP_SERVER_INCLUDED
