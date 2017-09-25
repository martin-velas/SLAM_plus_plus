/*
								+----------------------------------+
								|                                  |
								|    ***   Socket classes   ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2006   |
								|                                  |
								|             Socket.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __SOCKET_INCLUDED
#define __SOCKET_INCLUDED

/**
 *	@file Socket.h
 *	@author -tHE SWINe-
 *	@date 2006
 *	@brief Multiplatform socket class
 *
 *	@note This tends to produce the "First-chance exception 0x000006C5: The tag is invalid."
 *		when running under Win64/WOW64. That is a windows issue, not the issue with the code.
 *
 *	@date 2007-03-25
 *
 *	passed code revision
 *
 *	changed CStreamSocket::n_Read(), CStreamSocket::n_Write(), CDatagramSocket::n_SendTo()
 *	and CDatagramSocket::n_RecvFrom() to accept buffer size as unsigned int
 *
 *	@date 2007-04-04
 *
 *	passed code revision
 *
 *	added CSocket::GetPeerName() function to retrieve address of peer on the other side of socket
 *	renamed "TNativeAddress &TNetAddress::t_Address()" to "TNativeAddress &TNetAddress::r_Address()"
 *
 *	@date 2008-08-08
 *
 *	added \#ifdef for windows 64
 *
 *	@date 2009-05-04
 *
 *	fixed mixed windows / linux line endings
 *
 *	@date 2010-08-22
 *
 *	Changed CStreamSocket::n_Read(), CStreamSocket::n_Write(), CDatagramSocket::n_SendTo()
 *	and CDatagramSocket::n_RecvFrom() to accept buffer size as size_t.
 *
 *	Overhauled sockets implementation so windows and linux implementations share as much source as possible.
 *
 *	Added reference counter to CSocket, so sockets now may be copied.
 *
 *	Added functions for reading and writing all the data in the buffer, CStreamSocket::n_ReadAll()
 *	and CStreamSocket::n_WriteAll(), respectively.
 *
 *	Added socket timeout CSocket::SetTimeout() and functions for working with fd_set and select():
 *	CSocket::AddToSet(), CSocket::b_IsInSet(), CSocket::RemoveFromSet(), CSocket::n_Max_SetSize(),
 *	CSocket::t_EmptySet() and CStreamSocket::MakeTunnel().
 *
 *	@date 2010-10-29
 *
 *	Unified windows detection macro to "\#if defined(_WIN32) || defined(_WIN64)".
 *
 *	@date 2010-11-27
 *
 *	Fixed bug in CDatagramSocket::CDatagramSocket() (missing ! operator).
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 */

/**
 *	@def __SOCKET_H_COMPATIBILITY
 *	@brief enables deprecated aliases used by legacy applications
 */
//#define __SOCKET_H_COMPATIBILITY

#include "Integer.h"
#include "StlUtils.h"

#if defined(_WIN32) || defined(_WIN64)

#include <winsock.h>

/**
 *	@brief class for proper initialization and shutdown of winsock 2.0
 *	@note There is no such thing under linux.
 */
class CWinSock {
protected:
	bool m_b_ready; /**< @brief winsock initialization success flag */

public:
	/**
	 *	@brief startup
	 */
	CWinSock();

	/**
	 *	@brief cleanup
	 */
	~CWinSock();

	/**
	 *	@brief ready and supported?
	 *	@return Returns true on successful winsock initialization, otherwise returns false.
	 */
	bool b_Ready();
};

#else // WIN32, _WIN64

#include <unistd.h> // this is required in some environments
#include <netdb.h>
#include <sys/time.h> // fd_set
#include <netinet/in.h> // basic types
#include <sys/socket.h> // on BSD

#endif // WIN32, _WIN64

/**
 *	@brief socket address container for windows / linux
 */
struct TNetAddress {
public:
	typedef struct sockaddr_in TNativeAddress; /**< @brief internet address type */
#ifdef __SOCKET_H_COMPATIBILITY
	typedef TNativeAddress TAddrStruct; /**< @deprecated This is just another name for TNativeAddress, kept for backward compatibility. */
#endif // __SOCKET_H_COMPATIBILITY
	typedef struct hostent TNativeHost; /**< @brief internet address type */

protected:
	bool b_valid; /**< @brief valid address flag */
	TNativeAddress t_address; /**< @brief address */

public:
	/**
	 *	@brief default constructor
	 */
	inline TNetAddress()
		:b_valid(false)
	{}

	/**
	 *	@brief type-cast constructor
	 *
	 *	@param[in] _t_address is network address (is supposed to be valid)
	 */
	inline TNetAddress(TNativeAddress _t_address)
		:b_valid(true), t_address(_t_address)
	{}

	/**
	 *	@brief constructor with host name and port
	 *
	 *	@param[in] p_s_host_name is host name (in case it's 0, INADDR_ANY is used)
	 *
	 *	@note In case host name is not 0, gethostbyname() is called, which may fail
	 *		so it is recommended to call b_Valid().
	 */
	TNetAddress(const char *p_s_host_name, int n_port);

	/**
	 *	@brief looks for a specified host
	 *	@param[in] p_s_host_name is host name
	 *	@return Returns pointer to filled struct hostent, or 0 on failure.
	 */
	static TNativeHost *p_FindHost(const char *p_s_host_name);

	/**
	 *	@brief returns address validity flag is set by constructor
	 *	@return Returns address validity flag (true if valid, false if invalid).
	 */
	inline bool b_Valid() const { return b_valid; }

	/**
	 *	@brief gets network address
	 *	@return Returns network address in native form.
	 */
	inline const TNativeAddress &t_Address() const { return t_address; }

	/**
	 *	@brief gets network address
	 *	@return Returns network address in native form.
	 */
	inline TNativeAddress &r_Address() { return t_address; }

	/**
	 *	@brief converts the address to a string
	 *	@param[in,out] r_s_storage is storage to hold the string, contains the address upon return
	 *	@return Returns network address as a string, or 0 if there is not enough memory.
	 */
	const char *p_s_ToString(std::string &r_s_storage) const
	{
		uint32_t n_ip;
		memcpy(&n_ip, &t_address.sin_addr, sizeof(uint32_t));
		n_ip = ntohl(n_ip); // !!
		if(!stl_ut::Format(r_s_storage, "%d.%d.%d.%d:%d", n_ip >> 24, (n_ip >> 16) & 0xff,
		   (n_ip >> 8) & 0xff, n_ip & 0xff, ntohs(t_address.sin_port)))
			return 0;
		return r_s_storage.c_str();
	}
};

#ifdef __SOCKET_H_COMPATIBILITY
typedef TNetAddress TAddress; /**< @deprecated legacy name */
#endif // __SOCKET_H_COMPATIBILITY

/**
 *	@brief base socket class
 *
 *	@note There is no meaning in creating instances of this class
 *		  use CStreamSocket for stream-based sockets or CDatagramSocket
 *		  for datagram based sockets.
 */
class CSocket {
protected:
#if defined(_WIN32) || defined(_WIN64)
	static CWinSock m_startup; /**< @brief */
	typedef SOCKET TNativeSocket; /**< @brief native socket type */
#else // _WIN32 || _WIN64
	typedef int TNativeSocket; /**< @brief native socket type */
#endif // _WIN32 || _WIN64

	TNativeSocket m_n_socket; /**< @brief socket */
	size_t *m_p_ref_count; /**< @brief reference counter */

public:
	/**
	 *	@brief default constructor, sets no socket
	 */
	CSocket();

	/**
	 *	@brief copy-constructor
	 *
	 *	@param[in] r_socket is socket to be copied
	 */
	CSocket(const CSocket &r_socket);

	/**
	 *	@brief destructor, releases socket in case it was created
	 */
	~CSocket();

	/**
	 *	@brief copy-operator
	 *
	 *	@param[in] r_socket is socket to be copied
	 *
	 *	@return Returns reference to this.
	 */
	CSocket &operator =(const CSocket &r_socket);

	/**
	 *	@brief socket initialization check
	 *
	 *	@return Returns true in case socket was successfully created, otherwise false.
	 */
	bool b_Status() const;

	/**
	 *	@brief gets socket reference count (0 means socket is not created / was closed)
	 *
	 *	@return Returns socket reference count.
	 **/
	size_t n_Reference_Num() const;

	/**
	 *	@brief gets address of peer on the other side of the socket
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool GetPeerName(TNetAddress &r_t_address) const;

	/**
	 *	@brief sets (rather OS-specific) socket options
	 *
	 *	@return Returns true in case function succeeded, otherwise false. Always returns false in case no socket was created.
	 */
	bool SetSockOpt(int n_level, int n_name, const void *p_value, int n_value_size);

	/**
	 *	@brief closes socket
	 *
	 *	@return In case socket was opened returns true. in case socket was closed
	 *		already (or has never been created) returns false.
	 */
	bool Close();

	/**
	 *	@brief sets socket non-blocking
	 *
	 *	@return Returns true in case it was succesfully completed, otherwise false.
	 *		Always returns false in case no socket was created.
	 *
	 *	@note In case one of p_Accept, Connect, n_Read or n_Write fails, it is necessary to check b_WouldBlock()
	 *		to see wheter that was an error or whether the operation was just cancelled in order to avoid
	 *		blocking program execuiton.
	 */
	bool SetNonBlocking();

	/**
	 *	@brief determines whether last i/o operation on socket was cancelled because of error, or because it would block
	 *
	 *	@return Returns true in case EWOULDBLOCK (or WSAEWOULDBLOCK under win) was set, otherwise false.
	 *
	 *	@note This flag means that recently failed socket operation didn't fail because of real error
	 *		but instead because performing it would mean blocking program execution (non-blocking sockets).
	 */
	static bool b_WouldBlock();

	/**
	 *	@brief sets socket timeout
	 *
	 *	@param[in] n_timeout_msec is timeout in seconds (zero means no timeout)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool SetTimeout(int n_timeout_sec);

	/**
	 *	@brief adds this socket to fd_set
	 *
	 *	@param[out] r_t_set is fd_set where socket should be added
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool AddToSet(fd_set &r_t_set);

	/**
	 *	@brief determines whether is this socket present in fd_set
	 *
	 *	@param[out] r_t_set is fd_set, checked for socket presence
	 *
	 *	@return Returns true in case the socket is present in the set, otherwise returns false.
	 */
	bool b_IsInSet(const fd_set &r_t_set);

	/**
	 *	@brief removes this socket from fd_set
	 *
	 *	@param[out] r_t_set is fd_set from where socket should be removed
	 *
	 *	@return Returns true in case the socket was removed from the set, false in case it wasn't present in the set.
	 */
	bool RemoveFromSet(fd_set &r_t_set);

	/**
	 *	@brief determines maximum fd_set size
	 *	@return Returns maximum fd_set size (FD_SETSIZE equivalent).
	 */
	static size_t n_Max_SetSize();

	/**
	 *	@brief gets empty fd_set
	 *	@return Returns new empty fd_set.
	 */
	static fd_set t_EmptySet();
};

/**
 *	@brief class for simple stream (TCP/IP) socket operation
 */
class CStreamSocket : public CSocket {
public:
	/*/
	 *	@brief default constructor
	 *
	 *	Creates a new socket (it is recommended to call b_Status() to determine wheter the new socket was created successfuly).
	 */
	CStreamSocket();

	/**
	 *	@brief creates socket, in case constructor failed, or in case socket was closed using Close(), has no effect in case socket is valid
	 *
	 *	@return Returns true on success (meaning socket is valid), false on failure.
	 *
	 *	@note To create a <b>new</b> socket, use cumbination of Close() and Create(), or use copy-constructor with new Socket().
	 */
	bool Create();

	/**
	 *	@brief attempts to connect to host given by r_t_address
	 *
	 *	@param[in] r_t_address is address to connect to
	 *
	 *	@return Returns true in case connection was established, otherwise false,
	 *		Always returns false in case no socket was created.
	 */
	bool Connect(const TNetAddress &r_t_address);

	/**
	 *	@brief binds socket to address r_t_address, begin listening for n_max_connections
	 *
	 *	@param[in] r_t_address is address to listen on
	 *	@param[in] n_max_connections is connection queue size
	 *
	 *	@return Returns true in case socket is listening, otherwise false. Always returns false in case no socket was created.
	 */
	bool Listen(const TNetAddress &r_t_address, int n_max_connections);

	/**
	 *	@brief accepts incoming connections; r_t_address is filled with network address of socket which is returned
	 *
	 *	@return	Returns a new socket, connected to the other side or 0 on failure.
	 *		Always returns 0 in case no socket was created.
	 *
	 *	@note This propably will return 0 in case Listen() was not successfuly called prior calling this function.
	 */
	CStreamSocket *p_Accept(TNetAddress &r_t_address) const;

	/**
	 *	@brief receives data from socket to buffer
	 *
	 *	@param[out] p_buffer is output buffer
	 *	@param[in] n_buffer_size is maximum amount of data to be read (in bytes), but less than that may be actually read
	 *
	 *	@return Returns number of bytes written into the buffer on success or -1 on failure.
	 *		Always returns -1 in case no socket was created.
	 *
	 *	@note In case there are no data for the socket, the function is by default waiting
	 *		until some arrives (can be overriden by calling SetNonBlocking()).
	 */
	int n_Read(void *p_buffer, size_t n_buffer_size) const;

	/**
	 *	@brief transmits data from buffer to socket
	 *
	 *	@param[in] p_buffer is buffer, containing data to be sent
	 *	@param[in] n_buffer_size is size of data to be sent, but less than that may be actually sent
	 *
	 *	@return Returns returns number of bytes transmitted into socket on success or -1 on failure.
	 *		Always returns -1 in case no socket was created.
	 *
	 *	@note In case the socket buffer is full or there is another reason why the
	 *		operation, canot be performed immediately the function is by default waiting
	 *		until it can proceed (can be overriden by calling SetNonBlocking()).
	 */
	int n_Write(const void *p_buffer, size_t n_buffer_size) const;

	/**
	 *	@brief receives data from socket to buffer
	 *
	 *	@param[out] p_buffer is output buffer
	 *	@param[in] n_buffer_size is amount of data to be read (in bytes)
	 *
	 *	@return Returns number of bytes written into the buffer on success or -1 on failure.
	 *		Always returns -1 in case no socket was created.
	 *
	 *	@note In case there are no data for the socket, the function is by default waiting
	 *		until some arrives (can be overriden by calling SetNonBlocking()).
	 */
	int n_ReadAll(void *p_buffer, size_t n_buffer_size) const;

	/**
	 *	@brief transmits data from buffer to socket
	 *
	 *	@param[in] p_buffer is buffer, containing data to be sent
	 *	@param[in] n_buffer_size is size of data to be sent
	 *
	 *	@return Returns returns number of bytes transmitted into socket on success or -1 on failure.
	 *		Always returns -1 in case no socket was created.
	 *
	 *	@note In case the socket buffer is full or there is another reason why the
	 *		operation, canot be performed immediately the function is by default waiting
	 *		until it can proceed (can be overriden by calling SetNonBlocking()).
	 */
	int n_WriteAll(const void *p_buffer, size_t n_buffer_size) const;

	/**
	 *	@brief connects two sockets by a tunnell, blocks execution for tunnel lifetime
	 *
	 *	@param[in] r_other is socket to be connected with this one
	 *	@param[in] n_timeout_secs is connection timeout, in seconds; in case no data is transmitted
	 *		in that time, the tunnel closes. Setting this parameter to 0 creates tunnel that is kept
	 *		alive until one of the sides closes connection.
	 *	@param[in] n_buffer_size is size of data buffer, in bytes
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note In case n_timeout_secs is zero, one of sockets must become invalid to close the tunnel.
	 *		This may be in some implementations misinterpreted as an error and function may return false.
	 *	@note This function will fail with non-blocking sockets.
	 */
	bool MakeTunnel(CStreamSocket &r_other, int n_timeout_secs, size_t n_buffer_size = 65536);

protected:
	/**
	 *	@brief constructor with socket specification (used by p_Accept())
	 *
	 *	@param[in] n_socket is socket number
	 *
	 *	@note In case constructor fails to alloc reference counter, socket n_socket is not destroyed!
	 */
	CStreamSocket(TNativeSocket n_socket);
};

/**
 *	@brief class for simple UDP socket operation
 */
class CDatagramSocket : public CSocket {
public:
	/**
	 *	@brief default constructor
	 *
	 *	Creates a new socket (it is recommended to call b_Status() to determine wheter the new socket was created successfuly).
	 */
	CDatagramSocket();

	/**
	 *	@brief creates socket in case constructor failed, or in case socket was closed using Close(); it has no effect in case socket is valid
	 *
	 *	@return Returns true on success (meaning socket is valid), false on failure.
	 *
	 *	@note To create a <b>new</b> socket, use cumbination of Close() and Create(), or use copy-constructor with new Socket().
	 */
	bool Create();

	/**
	 *	@brief binds the socket to address r_t_address
	 *
	 *	@param[in] r_t_address is address to bind socket to
	 *
	 *	@return Returns true on succes or false on failure. Always returns false in case no socket was created.
	 */
	bool Bind(const TNetAddress &r_t_address);

	/**
	 *	@brief sends n_buffer_size bytes to  socket from buffer
	 *
	 *	@param[in] r_t_address is address to send data to
	 *	@param[out] p_buffer is buffer containing data to be sent
	 *	@param[in] n_buffer_size is size of buffer, in bytes
	 *
	 *	@return Returns number of bytes sent on success or -1 on failure. Always returns -1 in case no socket was created.
	 *
	 *	@note In case the socket buffer is full or there is another reason why the
	 *		operation, canot be performed immediately the function is by default waiting
	 *		until it can proceed (can be overriden by calling SetNonBlocking()).
	 */
	int n_SendTo(const TNetAddress &r_t_address, const void *p_buffer, size_t n_buffer_size) const;

	/**
	 *	@brief receives n_buffer_size bytes from socket to buffer; r_t_address is set to address which sent the data
	 *
	 *	@param[in] r_t_address is address to receive data from
	 *	@param[out] p_buffer is buffer for incoming data
	 *	@param[in] n_buffer_size is size of buffer, in bytes
	 *
	 *	@return Returns number of bytes written to the buffer on success or -1 on failure.
	 *		Always returns -1 in case no socket was created,
	 *
	 *	@note In case the socket buffer is full or there is another reason why the
	 *		operation, canot be performed immediately the function is by default waiting
	 *		until it can proceed (can be overriden by calling SetNonBlocking()).
	 */
	int n_RecvFrom(TNetAddress &r_t_address, void *p_buffer, size_t n_buffer_size) const;
};

#endif // !__SOCKET_INCLUDED
