/*
								+---------------------------------+
								|                                 |
								|   ***   HTTP Downloader   ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2011  |
								|                                 |
								|         HTTP_Download.h         |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __HTTP_DOWNLOADER_INCLUDED
#define __HTTP_DOWNLOADER_INCLUDED

/**
 *	@file proto/HTTP_Download.h
 *	@date 2011
 *	@author -tHE SWINe-
 *	@brief HTTP Downloader
 *
 *	@note This tends to produce the "First-chance exception 0x000006C5: The tag is invalid."
 *		when running under Win64/WOW64. That is a windows issue, not the issue with the code.
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 */

/**
 *	@def __NO_THREADED_DOWNLOADER
 *	@brief disables compilation of CThreadedHTTPDownloader
 */
//#define __NO_THREADED_DOWNLOADER

#if (defined(_WIN32) || defined(_WIN64)) && !defined(_WIN32_WCE)

/**
 *	@def __DOWNLOADER_PREALLOCATE_FILE
 *	@brief preallocates files to avoid filesystem (NTFS) fragmentation
 *	@note this is only available under windows
 */
#define __DOWNLOADER_PREALLOCATE_FILE

#endif // (_WIN32 || _WIN64) && !_WIN32_WCE

/**
 *	@def __DOWNLOADER_NULL_CHECK
 *	@brief checks if file begins with long string of nulls (sometimes happens on rapidshare)
 */
#define __DOWNLOADER_NULL_CHECK

/**
 *	@def __DOWNLOADER_FORMAT_CHECK
 *	@brief checks if file begins with correct signature (common formats like rar, zip, ...)
 */
#define __DOWNLOADER_FORMAT_CHECK

#ifdef __DOWNLOADER_PREALLOCATE_FILE
#define NOMINMAX
#include <windows.h>
#endif // __DOWNLOADER_PREALLOCATE_FILE
#include "../Integer.h"
#include "HTTP_Client.h"

/**
 *	@brief a simple file receiver to be used with CHTTP_File
 */
class CFileReceiver {
protected:
	const char *m_p_s_output;
	uint64_t &m_r_n_counter, &m_r_n_filesize;
#ifdef __DOWNLOADER_PREALLOCATE_FILE
	HANDLE m_p_fw;
#else //__DOWNLOADER_PREALLOCATE_FILE
	FILE *m_p_fw;
#endif //__DOWNLOADER_PREALLOCATE_FILE

	CHTTP_File &m_r_http_file;
	bool &m_r_b_interrupt;
	bool &m_r_b_write_error;

#ifdef __DOWNLOADER_PREALLOCATE_FILE
	static HANDLE h_OpenPreallocateFile(const char *p_s_filename, int64_t n_preallocate_size);
	static size_t fwrite(const void *p_data, size_t unit, size_t count, HANDLE h);
	static int fclose(HANDLE h);
#endif //__DOWNLOADER_PREALLOCATE_FILE

#ifdef __DOWNLOADER_NULL_CHECK
	size_t m_n_null_check;

	enum {
		checked_NullSequence = 100 * 1024 // it is quite unlikely there would be so long sequence of nulls
	};
#endif //__DOWNLOADER_NULL_CHECK

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] p_s_output is output file name
	 *	@param[in] r_b_interrupt is download interrupt flag
	 *	@param[out] r_b_write_error is write error flag
	 *	@param[out] r_n_counter is counter of amount of data written
	 *	@param[in,out] r_n_filesize is file size hint
	 *	@param[in] r_http_file is reference to the HTTP file being downloaded
	 */
	CFileReceiver(const char *p_s_output, bool &r_b_interrupt, bool &r_b_write_error,
		uint64_t &r_n_counter, uint64_t &r_n_filesize, CHTTP_File &r_http_file);

	/**
	 *	@brief destructor
	 */
	~CFileReceiver();

	/**
	 *	@brief writes data to the file specified in constructor
	 *
	 *	@param[in] p_data is pointer to the downloaded data buffer
	 *	@param[in] n_size is size of data to write, in bytes
	 *
	 *	@return Returns true on success, false on failure or in case interrupt flag was raised.
	 */
	bool operator ()(const void *p_data, size_t n_size);
};

/**
 *	@brief A simple utility for downloading files using the HTTP protocol.
 */
class CHTTPDownloader {
public:
	enum {
		down_Idle = -2, /**< @brief the downloader is idle */
		down_Running = -1, /**< @brief the download is running */
		down_Success = 0, /**< @brief the download succeeded */
		down_Failure = 1, /**< @brief the download failed (connection error, 404, server error) */
		down_WriteError, /**< @brief the download failed because of an i/o error (no free space) */
		down_LowMemoryFailure, /**< @brief downloader failed to alloc necessary buffers */
		down_Interrupted /**< @brief the download was interrupted */
	};

protected:
	std::string m_s_filename, m_s_url, m_s_proxy, m_s_extra_get_params;
	int m_n_proxy_retry_num, m_n_timeout;
	bool m_b_use_post;

	bool m_b_write_err;
	uint64_t m_n_file_size;
	uint64_t m_n_downloaded;
	int m_n_http_status;
	bool m_b_interrupt;

	int m_n_tag;

	CHTTP_Header m_http_header;

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	CHTTPDownloader();

	/**
	 *	@brief downloads a file from the given url
	 *
	 *	@param[in] p_s_filename is output file name (in local filesystem)
	 *	@param[in] p_s_url is url of a file to be downloaded
	 *	@param[in] n_tag is tag (can be used to mark download tasks)
	 *
	 *	@return Returns one of down_* (down_Success on success).
	 */
	int n_Download(const char *p_s_filename, const char *p_s_url, int n_tag = 0);

	/**
	 *	@brief sets socket timeout
	 *	@param[in] n_timeout_seconds is timeout in seconds; setting -1 leaves CHTTP_Socket
	 *		default timeout (default), using 0 disables timeout (waits forever; not reccommended)
	 *	@note This is only effective for the next call to n_Download(). Parameters of running
	 *		downloads cannot be changed.
	 */
	void Set_Timeout(int n_timeout_seconds);

	/**
	 *	@brief gets socket timeout
	 *	@return Returns timeout in seconds; -1 means the default CHTTP_Socket will be used,
	 *		0 means no timeout will be used (the socket waits forever).
	 */
	int n_Timeount() const;

	/**
	 *	@brief decides between GET (default) and POST requests
	 *
	 *	@param[in] b_use_POST decides between GET request (false) or POST request (true)
	 *
	 *	@note Use this in conjunction with SetParams() to fill-out web forms.
	 *	@note This is only effective for the next call to n_Download(). Parameters of running
	 *		downloads cannot be changed.
	 */
	void Set_RequestType(bool b_use_POST);

	/**
	 *	@brief determines if HTTP request GET is to be used
	 *	@return Returns true if GET will be used, otherwise returns false.
	 */
	bool b_Use_GET() const;

	/**
	 *	@brief determines if HTTP request POST is to be used
	 *	@return Returns true if POST will be used, otherwise returns false.
	 */
	bool b_Use_POST() const;

	/**
	 *	@brief disables proxy server usage (default)
	 *	@note This is only effective for the next call to n_Download(). Parameters of running
	 *		downloads cannot be changed.
	 */
	void Set_NoProxy();

	/**
	 *	@brief sets proxy server address
	 *
	 *	@param[in] p_s_proxy is proxy server url
	 *	@param[in] n_retry_num is number of download retries (for unreliable proxies)
	 *
	 *	@return Returns true on success, false on failure (not enough memory).
	 *
	 *	@note This is only effective for the next call to n_Download(). Parameters of running
	 *		downloads cannot be changed.
	 */
	bool Set_Proxy(const char *p_s_proxy, int n_retry_num = 1);

	/**
	 *	@brief gets proxy server address
	 *	@return Returns proxy server address, or 0 in case no proxy is used.
	 */
	const char *p_s_Proxy() const;

	/**
	 *	@brief gets number of download retries for proxy servers
	 *	@return Returns number of download retries.
	 */
	int n_Proxy_Retry_Num() const;

	/**
	 *	@brief sets additional HTTP request parameters
	 *	@param[in] p_s_extra_get_params is a string to be appended to the HTTP request (or 0)
	 *	@return Returns true on success, false on failure (not enough memory).
	 *	@note This is only effective for the next call to n_Download(). Parameters of running
	 *		downloads cannot be changed.
	 */
	bool Set_RequestParams(const char *p_s_extra_get_params);

	/**
	 *	@brief gets additional HTTP request parameters
	 *	@return Returns the additional HTTP request parameters (an empty string by default).
	 */
	const char *p_s_RequestParameters();

	/**
	 *	@brief gets last request output filename
	 *	@return Returns the path to the last downloaded file.
	 *	@note The file itself may not exist.
	 */
	const char *p_s_FileName() const;

	/**
	 *	@brief gets last request output filename
	 *	@return Returns the path to the last downloaded file.
	 *	@note The file itself may not exist.
	 */
	const std::string &r_s_FileName() const;

	/**
	 *	@brief gets last request url
	 *	@return Returns the url of the last request.
	 */
	const char *p_s_URL() const;

	/**
	 *	@brief gets last request url
	 *	@return Returns the url of the last request.
	 */
	const std::string &r_s_URL() const;

	/**
	 *	@brief gets size of the last requested file
	 *	@return Returns size of the last requested file (in bytes).
	 */
	uint64_t n_File_Size() const;

	/**
	 *	@brief gets amount of data downloaded of the last requested file
	 *	@return Returns amount of data downloaded (in bytes).
	 *	@note This updates as the file is being received and so it can be used
	 *		to estimate download speed and / or to display the download progress.
	 */
	uint64_t n_Downloaded_Size() const;

	/**
	 *	@brief gets HTTP response code of the last request
	 *	@return Returns HTTP response code of the last request (one of http_*, eg. http_OK).
	 */
	int n_HTTP_ResponseCode() const;

	/**
	 *	@brief gets HTTP response header
	 *
	 *	This may be useful to get eg. "Content-Type" field for received data
	 *	(it contains eg. "text/plain; charset=iso-8859-1").
	 *
	 *	@return Returns refererence to a HTTP response header of the last request.
	 *
	 *	@note The header may not be valid if the downloader fails with down_LowMemoryFailure.
	 *		If the downloader succeeds, the header is guaranteed to be valid.
	 *	@note This is only 
	 */
	const CHTTP_Header &r_HTTP_Header() const;

	/**
	 *	@brief gets tag of the last request
	 *	@return Returns tag of the last request.
	 */
	int n_Tag() const;

protected:
	void Reset_DownloadStatus();
	int n_Download_FILE();
};


#ifndef __NO_THREADED_DOWNLOADER

#include "../Thread.h"

/**
 *	@brief asynchronous version of CHTTPDownloader
 *
 *	This introduces the concept of "assignment". In case the download started
 *	successfully, the downloader is marked as assigned, and remains so until
 *	the download result is not obtained using n_GetResult() (except if it
 *	returns down_Running). Note this refers to result in the context of success
 *	or failure, not to the data being downloaded.
 *	Downloaders that are marked as assigned can not start new downloads, therefore
 *	the assignment is making the n_Download() and Start_Download() functions thread-safe.
 *	On the other hand, the Set_*() functions inherited from CHTTPDownloader are not.
 *
 *	@note This is not available if __NO_THREADED_DOWNLOADER is defined.
 */
class CThreadedHTTPDownloader : public CHTTPDownloader, public CRunable {
protected:
	CThread m_thread;
	CSemaphore m_assignment_sema;
	bool m_b_assignment;
	int m_n_result;

public:
	/**
	 *	@brief default constructor; has no effect
	 */
	CThreadedHTTPDownloader();

	/**
	 *	@brief determines if this downloader is assigned
	 *	@return Returns true if this downloader is assigned, otherwise returns false.
	 */
	bool b_IsAssigned() const;

	/**
	 *	@brief determines whether the download is running
	 *	@return Returns true if the download is running, otherwise returns false.
	 */
	bool b_IsRunning() const;

	/**
	 *	@brief downloads a file from the given url and waits for completion
	 *
	 *	@param[in] p_s_filename is output file name (in local filesystem)
	 *	@param[in] p_s_url is url of a file to be downloaded
	 *	@param[in] n_tag is tag (can be used to mark download tasks)
	 *	@param[out] p_b_was_assigned is pointer to assignment flag (if the downloader
	 *		was assigned at the time of the call, it is set, otherwise is cleared;
	 *		can be null)
	 *
	 *	@return Returns one of down_* (down_Success on success).
	 *
	 *	@note This marks this downloader as assigned, so it can't be used by other
	 *		threads until this function returns and n_GetResult() is called with
	 *		return value other than down_Running.
	 *	@note This runs in the current thread (no new thread is started).
	 *	@note In case this downloader is assigned, returns down_Failure immediately.
	 */
	int n_Download(const char *p_s_filename, const char *p_s_url,
		int n_tag = 0, bool *p_b_was_assigned = 0);

	/**
	 *	@brief downloads a file from the given url and waits for completion
	 *
	 *	@param[in] p_s_filename is output file name (in local filesystem)
	 *	@param[in] r_s_url is url of a file to be downloaded as std::string
	 *	@param[in] n_tag is tag (can be used to mark download tasks)
	 *	@param[out] p_b_was_assigned is pointer to assignment flag (if the downloader
	 *		was assigned at the time of the call, it is set, otherwise is cleared;
	 *		can be null)
	 *
	 *	@return Returns one of down_* (down_Success on success).
	 *
	 *	@note This marks this downloader as assigned, so it can't be used by other
	 *		threads until this function returns and n_GetResult() is called with
	 *		return value other than down_Running.
	 *	@note This runs in the current thread (no new thread is started).
	 *	@note In case this downloader is assigned, returns down_Failure immediately.
	 */
	inline int n_Download(const char *p_s_filename, const std::string &r_s_url,
		int n_tag = 0, bool *p_b_was_assigned = 0)
	{
		return n_Download(p_s_filename, r_s_url.c_str(), n_tag, p_b_was_assigned);
	}

	/**
	 *	@brief starts download of a file from the given url and returns immediately
	 *
	 *	@param[in] p_s_filename is output file name (in local filesystem)
	 *	@param[in] p_s_url is url of a file to be downloaded
	 *	@param[in] n_tag is tag (can be used to mark download tasks)
	 *	@param[out] p_b_was_assigned is pointer to assignment flag (if the downloader
	 *		was assigned at the time of the call, it is set, otherwise is cleared;
	 *		can be null)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This marks this downloader as assigned, so it can't be used by other
	 *		threads until this function returns and n_GetResult() is called with
	 *		return value other than down_Running.
	 *	@note In case this downloader is already assigned, returns false immediately.
	 */
	bool Start_Download(const char *p_s_filename, const char *p_s_url,
		int n_tag = 0, bool *p_b_was_assigned = 0);

	/**
	 *	@brief starts download of a file from the given url and returns immediately
	 *
	 *	@param[in] p_s_filename is output file name (in local filesystem)
	 *	@param[in] r_s_url is url of a file to be downloaded as std::string
	 *	@param[in] n_tag is tag (can be used to mark download tasks)
	 *	@param[out] p_b_was_assigned is pointer to assignment flag (if the downloader
	 *		was assigned at the time of the call, it is set, otherwise is cleared;
	 *		can be null)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This marks this downloader as assigned, so it can't be used by other
	 *		threads until this function returns and n_GetResult() is called with
	 *		return value other than down_Running.
	 *	@note In case this downloader is already assigned, returns false immediately.
	 */
	inline bool Start_Download(const char *p_s_filename, const std::string &r_s_url,
		int n_tag = 0, bool *p_b_was_assigned = 0)
	{
		return Start_Download(p_s_filename, r_s_url.c_str(), n_tag, p_b_was_assigned);
	}

	/**
	 *	@brief interrupts the downloading process, even if it didn't complete
	 *
	 *	@param[in] b_wait_for_thread enables the caller to wait for the thread
	 *		running the downloading procedure to complete
	 *
	 *	@return Returns one of down_*, typically down_Running (b_wait_for_thread was
	 *		not set and the download was still running), down_Interrupted (the download was
	 *		interrupted) or down_Failed / down_Finished (the download managed to fail / finish
	 *		before it was interrupted).
	 *
	 *	@note It takes some time for the download to be interrupted. The time needed to do so
	 *		varies between zero and socket timeout that was set for this request.
	 */
	int n_Stop(bool b_wait_for_thread);

	/**
	 *	@brief waits for the running download to finish (or returns immediately
	 *		if the download is not running)
	 *	@return Returns true on success, false on failure (CSemaphore::Wait() failed,
	 *		shouldn't normally happen).
	 */
	bool WaitForFinish();

	/**
	 *	@brief gets downloader status
	 *	@return Returns one of down_*.
	 */
	int n_PeekResult() const;

	/**
	 *	@brief gets downloader status
	 *	@param[in] b_wait_for_finish_if_running enables the caller to wait for the thread
	 *		running the downloading procedure to complete
	 *	@return Returns one of down_*, typically down_Running (b_wait_for_finish_if_running was
	 *		not set and the download is still running), down_Interrupted (the download was
	 *		interrupted by calling n_Stop()) or down_Failed / down_Finished (the download
	 *		finished).
	 *	@note If this function returns down_Running, the downloader remains assigned
	 *		and n_Download() / Start_Download() can not be successfully called. Otherwise
	 *		the downloader is unassigned and can be reused.
	 */
	int n_GetResult(bool b_wait_for_finish_if_running);

	/**
	 *	@brief chooses a free one or at least estimates the next downloader to finish
	 *
	 *	@param[in] p_begin_it is (const) iterator pointing to the first downloader
	 *	@param[in] p_end_it is (const) iterator pointing to the element after the last downloader
	 *
	 *	@return Returns iterator in range [p_begin_it, p_end_it) pointing to a chosen downloader.
	 */
	template <class _It>
	static _It p_Select(_It p_begin_it, _It p_end_it)
	{
		uint64_t n_min_remaining_size = UINT64_MAX;
		_It p_next_to_finish_it = p_end_it; // choose the last one as the next one to finish

		_It p_random_it = p_begin_it; // choose a first iterator as random

		for(size_t n_count = 1; p_begin_it != p_end_it; ++ p_begin_it, ++ n_count) {
			const CThreadedHTTPDownloader &r_downloader = *p_begin_it;
			if(!r_downloader.b_IsRunning())
				return p_begin_it;
			// found one that is not running
			
			if(r_downloader.n_File_Size()) {
				uint64_t n_remains = r_downloader.n_File_Size() -
					r_downloader.n_Downloaded_Size();
				if(n_min_remaining_size > n_remains) {
					n_min_remaining_size = n_remains;
					p_next_to_finish_it = p_begin_it;
				}
				break; // no need to choose random anymore
			}
			// found one that is known to finish in (theoretically) finite and comparable time

			if(rand() % n_count == 0)
				p_random_it = p_begin_it;
			// choose one at random (we don't know the length of p_end_it p_begin_it as it may be a linked list)
		}

		if(p_next_to_finish_it == p_end_it)
			return p_random_it;
		// in case there is none that seems to be downloading something, choose one at random

		for(; p_begin_it != p_end_it; ++ p_begin_it) {
			const CThreadedHTTPDownloader &r_downloader = *p_begin_it;
			if(!r_downloader.b_IsRunning())
				return p_begin_it;
			// found one that is not running
			
			if(r_downloader.n_File_Size()) {
				uint64_t n_remains = r_downloader.n_File_Size() -
					r_downloader.n_Downloaded_Size();
				if(n_min_remaining_size > n_remains) {
					n_min_remaining_size = n_remains;
					p_next_to_finish_it = p_begin_it;
				}
			}
			// found one that is known to finish in (theoretically) finite and comparable time
		}
		// the same loop, but is not choosing random one anymore as we will not need it

		_ASSERTE(p_next_to_finish_it != p_end_it);
		return p_next_to_finish_it;
		// choose the one that is going to finish the first
		// (assuming all downloaders are running at constant download speed,
		// none of them will fail and none of those that do not yet know received file
		// size will get smaller file than the smallest remaining size)
	}

protected:
	virtual void Run();
};

#endif // !__NO_THREADED_DOWNLOADER

#endif // !__HTTP_DOWNLOADER_INCLUDED
