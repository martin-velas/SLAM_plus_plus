/*
								+----------------------------------+
								|                                  |
								|    ***   Socket classes   ***    |
								|                                  |
								|   Copyright © -tHE SWINe- 2006   |
								|                                  |
								|            Socket.cpp            |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file Socket.cpp
 *	@author -tHE SWINe-
 *	@date 2006
 *	@brief Multiplatform socket class
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
 */

#include "NewFix.h"
#include "CallStack.h"
#include "Integer.h"

#if defined(_WIN32) || defined(_WIN64)

#include <winsock.h>
#include "Socket.h"

#define b_ValidSocket(s) ((s) != INVALID_SOCKET)

typedef int __address_length_t__;
typedef const char *__sockopt_value_t__;

CWinSock::CWinSock()
	:m_b_ready(false)
{
	WORD n_min_version = MAKEWORD(2, 0);
	WSADATA t_winsock_info;

	if(WSAStartup(n_min_version, &t_winsock_info)) {
		WSACleanup();
		return;
	}

	if(t_winsock_info.wVersion != n_min_version) {
		WSACleanup();
		return;
	}
	// check WinSock version

	m_b_ready = true;
}

bool CWinSock::b_Ready()
{
	return m_b_ready;
}

CWinSock::~CWinSock()
{
	WSACleanup();
}

/*
 *								=== ~CWinSock ===
 */

#else // _WIN32 || _WIN64

#include "Socket.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <netdb.h>
#include <errno.h>
#include <sys/time.h> // struct timeval

#define INVALID_SOCKET -1
// linux doesn't have this

#define b_ValidSocket(s) ((s) >= 0)

typedef struct hostent *LPHOSTENT;
// linux doesn't have this

#define closesocket(s) close(s)
// winsock naming

typedef unsigned int __address_length_t__;
typedef const void *__sockopt_value_t__;

#endif // _WIN32 || _WIN64

/*
 *								=== TNetAddress ===
 */

TNetAddress::TNetAddress(const char *p_s_host_name, int n_port)
	:b_valid(false)
{
	memset(&t_address, 0, sizeof(TNativeAddress));
	t_address.sin_family = AF_INET;
	t_address.sin_port = htons(n_port);
	if(p_s_host_name) {
		TNativeHost *p_host;
		if(!(p_host = p_FindHost(p_s_host_name)))
			return;
#if defined(_WIN32) || defined(_WIN64)
		t_address.sin_addr = *((LPIN_ADDR)*p_host->h_addr_list);
#else // _WIN32 || _WIN64
		memcpy(&t_address.sin_addr, p_host->h_addr, p_host->h_length);
#endif // _WIN32 || _WIN64
	} else
		t_address.sin_addr.s_addr = INADDR_ANY;
	b_valid = true;
}

TNetAddress::TNativeHost *TNetAddress::p_FindHost(const char *p_s_host_name)
{
	TNativeHost *p_host;
	if((p_host = gethostbyname(p_s_host_name)) == NULL)
		return 0;
	return p_host;
}

/*
 *								=== ~TNetAddress ===
 */

/*
 *								=== CSocket ===
 */

#if defined(_WIN32) || defined(_WIN64)
CWinSock CSocket::m_startup;
#endif // _WIN32 || _WIN64

CSocket::CSocket()
	:m_n_socket(INVALID_SOCKET), m_p_ref_count(0) // null, not a new size_t containing null!
{}

CSocket::CSocket(const CSocket &r_socket)
	:m_n_socket(r_socket.m_n_socket), m_p_ref_count(r_socket.m_p_ref_count)
{
	if(!b_Status()) {
		_ASSERTE(!b_ValidSocket(m_n_socket) && !m_p_ref_count);
	} else {
		_ASSERTE(b_ValidSocket(m_n_socket) && m_p_ref_count);
		++ *m_p_ref_count;
	}
}

CSocket &CSocket::operator =(const CSocket &r_socket)
{
	if(&r_socket == this)
		return *this;
	// handle self-assignment

	Close();
	// close this socket

	m_n_socket = r_socket.m_n_socket;
	m_p_ref_count = r_socket.m_p_ref_count;

	if(!b_Status()) {
		m_n_socket = INVALID_SOCKET;
		m_p_ref_count = 0;
	} else {
		_ASSERTE(m_p_ref_count);
		++ *m_p_ref_count;
	}

	return *this;
}

CSocket::~CSocket()
{
	Close();
}

bool CSocket::b_Status() const
{
	_ASSERTE((!b_ValidSocket(m_n_socket) && !m_p_ref_count) ||
		(b_ValidSocket(m_n_socket) && m_p_ref_count && *m_p_ref_count));
	// either socket is invalid and there are no references, or it's valid and reference count is nonzero

	return /*m_p_ref_count && *m_p_ref_count > 0 &&*/ b_ValidSocket(m_n_socket);
}

size_t CSocket::n_Reference_Num() const
{
	_ASSERTE((!b_ValidSocket(m_n_socket) && !m_p_ref_count) ||
		(b_ValidSocket(m_n_socket) && m_p_ref_count && *m_p_ref_count));
	// either socket is invalid and there are no references, or it's valid and reference count is nonzero

	return (m_p_ref_count)? *m_p_ref_count : 0;
}

bool CSocket::Close()
{
	_ASSERTE((!b_ValidSocket(m_n_socket) && !m_p_ref_count) ||
		(b_ValidSocket(m_n_socket) && m_p_ref_count && *m_p_ref_count));
	// either socket is invalid and there are no references, or it's valid and reference count is nonzero

	if(m_p_ref_count) {
		if(!(-- *m_p_ref_count)) {
			_ASSERTE(b_ValidSocket(m_n_socket));
			closesocket(m_n_socket);
			delete m_p_ref_count;
		}

		m_n_socket = INVALID_SOCKET;
		m_p_ref_count = 0; // this belongs to different socket now

		return true;
	} else
		return false;
	// decrement reference count
}

bool CSocket::GetPeerName(TNetAddress &r_t_address) const
{
	__address_length_t__ n_addr_len = sizeof(TNetAddress::TNativeAddress);
	return b_Status() && !getpeername(m_n_socket, (struct sockaddr*)&r_t_address.t_Address(), &n_addr_len);
}

bool CSocket::SetSockOpt(int n_level, int n_name, const void *p_value, int n_value_size)
{
	return b_Status() && !setsockopt(m_n_socket, n_level, n_name, (__sockopt_value_t__)p_value, n_value_size);
}

#if defined(_WIN32) || defined(_WIN64)

bool CSocket::SetNonBlocking()
{
	if(!b_Status())
		return false;
	unsigned long n_true = 1;
	if(ioctlsocket(m_n_socket, FIONBIO, &n_true) < 0) // works nicely
		return false;
	/*int n_flags;
	if((n_flags = fcntl(m_n_socket, F_GETFL, 0)) < 0 ||
	   fcntl(m_n_socket, F_SETFL, n_flags | O_NONBLOCK) < 0) // this is how it is done under windows
		return false;*/
	return true;
}

bool CSocket::b_WouldBlock()
{
	return WSAGetLastError() == WSAEWOULDBLOCK; // this is how it is done under windows
}

bool CSocket::SetTimeout(int n_timeout_sec)
{
	n_timeout_sec *= 1000;
	return !SetSockOpt(SOL_SOCKET, SO_RCVTIMEO, &n_timeout_sec, sizeof(int)); // this is how it is done under windows
}

#else // _WIN32 || _WIN64

bool CSocket::SetNonBlocking()
{
	if(!b_Status())
		return false;
	int n_flags;
	if((n_flags = fcntl(m_n_socket, F_GETFL, 0)) < 0 ||
	   fcntl(m_n_socket, F_SETFL, n_flags | O_NONBLOCK) < 0) // this is how it is done under linux
		return false;
	return true;
}

bool CSocket::b_WouldBlock()
{
	return errno == EWOULDBLOCK; // this is how it is done under linux
}

bool CSocket::SetTimeout(int n_timeout_sec)
{
	struct timeval tv;
	tv.tv_sec = n_timeout_sec;
	tv.tv_usec = 0;
	return !SetSockOpt(SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); // this is how it is done under linux
}

#endif // _WIN32 || _WIN64

bool CSocket::AddToSet(fd_set &r_t_set)
{
	fd_set *p_set = &r_t_set;

	if(!b_ValidSocket(m_n_socket))
		return false;
	// socket is not valid

	if(b_IsInSet(r_t_set))
		return true;
	// it's already there

#if (!defined(_WIN32) && !defined(_WIN64)) || defined(FD_SET)
	FD_SET(m_n_socket, p_set); // this is how it is done under linux/windows
	// returns no value
#else // (!_WIN32 && !_WIN64) || FD_SET
	p_set->fd_array[p_set->fd_count] = m_n_socket; // this is how it can be done under windows
	++ p_set->fd_count;
	// adds socket to the set
#endif // (!_WIN32 && !_WIN64) || FD_SET

	return true;
}

bool CSocket::RemoveFromSet(fd_set &r_t_set)
{
	fd_set *p_set = &r_t_set;

	if(!b_ValidSocket(m_n_socket))
		return false;
	// socket is not valid

#if (!defined(_WIN32) && !defined(_WIN64)) || (defined(FD_ISSET) && defined(FD_CLR))
	if(FD_ISSET(m_n_socket, p_set)) {
		FD_CLR(m_n_socket, p_set); // this is how it is done under linux/windows
		return true;
	}
	// remove socket from the set
#else // (!_WIN32 && !_WIN64) || (FD_ISSET && FD_CLR)
	for(size_t i = 0; i < p_set->fd_count; ++ i) {
		if(p_set->fd_array[i] == m_n_socket) { // this is how it can be done under windows
			for(size_t j = i + 1; j < p_set->fd_count; ++ j)
				p_set->fd_array[j - 1] = p_set->fd_array[j];
			-- p_set->fd_count;
			return true;
		}
	}
	// finds socket in set, and removes it
#endif // (!_WIN32 && !_WIN64) || (FD_ISSET && FD_CLR)

	return false;
}

bool CSocket::b_IsInSet(const fd_set &r_t_set)
{
	const fd_set *p_set = &r_t_set;

	if(!b_ValidSocket(m_n_socket))
		return false;
	// socket is not valid

#if (!defined(_WIN32) && !defined(_WIN64)) || defined(FD_ISSET)
	if(FD_ISSET(m_n_socket, p_set)) // this is how it is done under linux/windows
		return true;
	// find socket in the set
#else // (!_WIN32 && !_WIN64) || FD_ISSET
	for(size_t i = 0; i < p_set->fd_count; ++ i) { // this is how it can be done under windows
		if(p_set->fd_array[i] == m_n_socket)
			return true;
	}
	// finds socket in set
#endif // (!_WIN32 && !_WIN64) || FD_ISSET

	return false;
}

size_t CSocket::n_Max_SetSize()
{
	return FD_SETSIZE;
}

fd_set CSocket::t_EmptySet()
{
	fd_set t_set;
	FD_ZERO(&t_set);
	return t_set;
}

/*
 *								=== ~CSocket ===
 */

/*
 *								=== CStreamSocket ===
 */

CStreamSocket::CStreamSocket()
	:CSocket()
{
	_ASSERTE(!m_p_ref_count && !b_ValidSocket(m_n_socket));
	// makes sure there's no socket

	if(!(m_p_ref_count = new(std::nothrow) size_t))
		return;
	// alloc reference counter

	if(!b_ValidSocket(m_n_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP))) {
		delete m_p_ref_count;
		m_p_ref_count = 0; // !!
		// free reference counter on failure

		return;
	}
	// create socket

	*m_p_ref_count = 1;
	// we have one reference
}

CStreamSocket::CStreamSocket(TNativeSocket n_socket)
	:CSocket()
{
	_ASSERTE(!m_p_ref_count && !b_ValidSocket(m_n_socket));
	// makes sure there's no socket

	if(!(m_p_ref_count = new(std::nothrow) size_t))
		return;
	// alloc reference counter

	if(!b_ValidSocket(m_n_socket = n_socket)) {
		delete m_p_ref_count;
		m_p_ref_count = 0; // !!
		// free reference counter on failure

		return;
	}
	// create socket

	*m_p_ref_count = 1;
	// we have one reference
}

bool CStreamSocket::Create()
{
	if(b_Status())
		return true;
	// make sure socket is closed

	if(!(m_p_ref_count = new(std::nothrow) size_t))
		return false;
	// alloc reference counter

	if(!b_ValidSocket(m_n_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP))) {
		delete m_p_ref_count;
		m_p_ref_count = 0; // !!
		// free reference counter on failure

		return false;
	}
	// create socket

	*m_p_ref_count = 1;
	// we have one reference

	return true;
}

bool CStreamSocket::Connect(const TNetAddress &r_t_address)
{
	return b_Status() && !connect(m_n_socket, (const struct sockaddr*)&r_t_address.t_Address(), sizeof(TNetAddress::TNativeAddress));
}

bool CStreamSocket::Listen(const TNetAddress &r_t_address, int n_max_connections)
{
	return b_Status() &&
		!bind(m_n_socket, (const struct sockaddr*)&r_t_address.t_Address(), sizeof(TNetAddress::TNativeAddress)) &&
		!listen(m_n_socket, n_max_connections);
}

CStreamSocket *CStreamSocket::p_Accept(TNetAddress &r_t_address) const
{
	TNativeSocket n_socket;
    __address_length_t__ n_addr_len = sizeof(struct sockaddr_in);
	if(!b_ValidSocket(n_socket = accept(m_n_socket, (struct sockaddr*)&r_t_address.t_Address(), &n_addr_len)))
		return 0;
	// maybe just timeout or something ... do not disconnect me yet

	CStreamSocket *p_new;
	if(!(p_new = new(std::nothrow) CStreamSocket(n_socket)))
		return 0;
	// assign socket

	return p_new;
}

#if defined(_WIN32) || defined(_WIN64)

int CStreamSocket::n_Read(void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	int n_result = recv(m_n_socket, (char*)p_buffer, (int)n_buffer_size, 0);
	// If no error occurs, recv returns the number of bytes received ...
	// If the connection has been gracefully closed, the return value is zero.
	// Otherwise, a value of SOCKET_ERROR is returned ...

	if(n_result == SOCKET_ERROR)
		return -1;
	return n_result;
}

int CStreamSocket::n_Write(const void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	int n_result = send(m_n_socket, (const char*)p_buffer, (int)n_buffer_size, 0);
	// If no error occurs, send returns the total number of bytes sent, which
	// can be less than the number requested to be sent in the len parameter.
	// Otherwise, a value of SOCKET_ERROR is returned ...

	if(n_result == SOCKET_ERROR)
		return -1;
	return n_result;
}

#else // _WIN32 || _WIN64

int CStreamSocket::n_Read(void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	return read(m_n_socket, (char*)p_buffer, n_buffer_size);
	// Upon successful completion, read(), pread() and readv() return a non-negative integer indicating
	// the number of bytes actually read. Otherwise, the functions return -1 and set errno to indicate the error.
}

int CStreamSocket::n_Write(const void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	return write(m_n_socket, (const char*)p_buffer, n_buffer_size);
	// Upon successful completion, write() and pwrite() shall return the number of bytes actually written
	// to the file associated with fildes. This number shall never be greater than nbyte. Otherwise, -1 shall
	// be returned and errno set to indicate the error.
}

#endif // _WIN32 || _WIN64

int CStreamSocket::n_ReadAll(void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	uint8_t *p_buffer_ptr = (uint8_t*)p_buffer;
	size_t n_read = 0;
	while(n_buffer_size) {
		int n_read_now;
		if((n_read_now = n_Read(p_buffer_ptr, n_buffer_size)) < 0)
			return n_read_now;
		if(!n_read_now)
			break; // connection closed
		p_buffer_ptr += n_read_now;
		n_buffer_size -= n_read_now;
		n_read += unsigned(n_read_now);
	}
	_ASSERTE(n_read <= INT_MAX);
	return int(n_read);
}

int CStreamSocket::n_WriteAll(const void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	const uint8_t *p_buffer_ptr = (const uint8_t*)p_buffer;
	size_t n_written = 0;
	while(n_buffer_size) {
		int n_written_now;
		if((n_written_now = n_Write(p_buffer_ptr, n_buffer_size)) < 0)
			return n_written_now;
		if(!n_written_now)
			break; // connection closed
		p_buffer_ptr += n_written_now;
		n_buffer_size -= n_written_now;
		n_written += unsigned(n_written_now);
	}
	_ASSERTE(n_written <= INT_MAX);
	return int(n_written);
}

bool CStreamSocket::MakeTunnel(CStreamSocket &r_other, int n_timeout_secs, size_t n_buffer_size)
{
	uint8_t *p_buffer;
	if(!(p_buffer = new(std::nothrow) uint8_t[n_buffer_size]))
		return false;
	// alloc buffer

	bool b_remain_open = !n_timeout_secs;
	// setting timeout to 0 means no timeout

	for(;;) {
		fd_set t_set = t_EmptySet();
		fd_set t_failset = t_EmptySet();
		if(!AddToSet(t_set) || !r_other.AddToSet(t_set) ||
		   !AddToSet(t_failset) || !r_other.AddToSet(t_failset)) {
			delete[] p_buffer;
			return false;
		}
		// create fd_set with sockets

		struct timeval t_time;
		t_time.tv_sec = (b_remain_open)? 1000 : n_timeout_secs;
		t_time.tv_usec = 0;
		int n_result;
		if((n_result = select(2, &t_set, 0, &t_failset, &t_time)) < 0) {
			if(b_remain_open)
				break;
			// this just probably means that one of the sockets closed connection

			delete[] p_buffer;
			return false;
		}
		if(b_IsInSet(t_failset) || r_other.b_IsInSet(t_failset))
			break; // exception on one of socket(s) occured (socket closed?) - closing tunnel
		if(!n_result || (!b_IsInSet(t_set) && !r_other.b_IsInSet(t_set))) {
			if(b_remain_open)
				continue;
			// in case tunnel is supposed to remain open, then just wait again

			break;
			// 0 means timeout
		}
		// call select() to see which socket has data ready to be read

		if(b_IsInSet(t_set)) {
			int n_read;
			if((n_read = n_Read(p_buffer, n_buffer_size)) < 0) {
				delete[] p_buffer;
				return false;
			}
			if(!n_read)
				break; // connection was closed
			// receive data from this socket

			if(r_other.n_WriteAll(p_buffer, n_read) != n_read) {
				delete[] p_buffer;
				return false;
			}
			// relay data to the other socket
		}
		// send data from this socket to r_other

		if(r_other.b_IsInSet(t_set)) {
			int n_read;
			if((n_read = r_other.n_Read(p_buffer, n_buffer_size)) < 0) {
				delete[] p_buffer;
				return false;
			}
			if(!n_read)
				break; // connection was closed
			// receive data from this socket

			if(n_WriteAll(p_buffer, n_read) != n_read) {
				delete[] p_buffer;
				return false;
			}
			// relay data to the other socket
		}
		// send data from r_other to this socket
	}

	delete[] p_buffer;
	// cleanup

	return true;
}

/*
 *								=== ~CStreamSocket ===
 */

/*
 *								=== CDatagramSocket ===
 */

CDatagramSocket::CDatagramSocket()
	:CSocket()
{
	_ASSERTE(!m_p_ref_count && !b_ValidSocket(m_n_socket));
	// makes sure there's no socket

	if(!(m_p_ref_count = new(std::nothrow) size_t))
		return;
	// alloc reference counter

	if(!b_ValidSocket(m_n_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP))) {
		delete m_p_ref_count;
		m_p_ref_count = 0; // !!
		// free reference counter on failure

		return;
	}
	// create socket

	*m_p_ref_count = 1;
	// we have one reference
}

bool CDatagramSocket::Create()
{
	if(b_Status())
		return true;
	// make sure socket is closed

	if(!(m_p_ref_count = new(std::nothrow) size_t))
		return false;
	// alloc reference counter

	if(!b_ValidSocket(m_n_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP))) {
		delete m_p_ref_count;
		m_p_ref_count = 0; // !!
		// free reference counter on failure

		return false;
	}
	// create socket

	*m_p_ref_count = 1;
	// we have one reference

	return true;
}

bool CDatagramSocket::Bind(const TNetAddress &r_t_address)
{
	return b_Status() && !bind(m_n_socket, (const struct sockaddr*)&r_t_address.t_Address(), sizeof(TNetAddress::TNativeAddress));
}

int CDatagramSocket::n_SendTo(const TNetAddress &r_t_address,
	const void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	int n_result = sendto(m_n_socket, (const char*)p_buffer, int(n_buffer_size), 0,
		(const sockaddr*)&r_t_address.t_Address(), sizeof(struct sockaddr));

#if defined(_WIN32) || defined(_WIN64)
	if(n_result == SOCKET_ERROR)
		return -1;
#else // _WIN32 || _WIN64
	if(n_result < 0)
		return -1;
#endif // _WIN32 || _WIN64

	return n_result;
}

int CDatagramSocket::n_RecvFrom(TNetAddress &r_t_address,
	void *p_buffer, size_t n_buffer_size) const
{
	_ASSERTE(n_buffer_size <= INT_MAX);
	__address_length_t__ n_addr_len = sizeof(TNetAddress::TNativeAddress);
	int n_result = recvfrom(m_n_socket, (char*)p_buffer, int(n_buffer_size), 0,
		(sockaddr*)&r_t_address.t_Address(), &n_addr_len);

#if defined(_WIN32) || defined(_WIN64)
	if(n_result == SOCKET_ERROR)
		return -1;
#else // _WIN32 || _WIN64
	if(n_result < 0)
		return -1;
#endif // _WIN32 || _WIN64

	return n_result;
}

/*
 *								=== ~CDatagramSocket ===
 */
