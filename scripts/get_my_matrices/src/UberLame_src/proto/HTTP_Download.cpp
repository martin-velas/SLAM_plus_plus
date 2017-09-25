/*
								+---------------------------------+
								|                                 |
								|   ***   HTTP Downloader   ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2011  |
								|                                 |
								|        HTTP_Download.cpp        |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file proto/HTTP_Download.cpp
 *	@date 2011
 *	@author -tHE SWINe-
 *	@brief HTTP Downloader
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include <vector>
#include <stdio.h>
#include "../MinMax.h"
#include "HTTP_Client.h"
#include "HTTP_Download.h"

#if defined(_WIN32) || defined(_WIN64)
#define strncasecmp(a,b,n) _strnicmp((a), (b), (n))
#define strcasecmp(a,b) _stricmp((a), (b))
#endif //_WIN32 || _WIN64

/*
 *								=== CFileReceiver ===
 */

#ifdef __DOWNLOADER_PREALLOCATE_FILE

HANDLE CFileReceiver::h_OpenPreallocateFile(const char *p_s_filename, int64_t n_preallocate_size)
{
	HANDLE h;
	if((h = CreateFile(p_s_filename, GENERIC_WRITE, 0, NULL,
	   CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE) {
		if((h = CreateFile(p_s_filename, GENERIC_WRITE, 0, NULL,
		   OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE)
			return INVALID_HANDLE_VALUE;
	}

	long n_hidword = long(n_preallocate_size >> 32);
	SetFilePointer(h, long(n_preallocate_size & 0xffffffff), &n_hidword, FILE_BEGIN);
	SetEndOfFile(h);
	SetFilePointer(h, 0, NULL, FILE_BEGIN);

	return h;
}

size_t CFileReceiver::fwrite(const void *p_data, size_t unit, size_t count, HANDLE h)
{
	DWORD n_written;
	_ASSERTE(unit * count <= ULONG_MAX); // DWORD == unsigned long
	if(!WriteFile(h, p_data, DWORD(unit * count), &n_written, NULL))
		return -1;
	return n_written / unit;
}

int CFileReceiver::fclose(HANDLE h)
{
	return CloseHandle(h);
}

#endif //__DOWNLOADER_PREALLOCATE_FILE

CFileReceiver::CFileReceiver(const char *p_s_output, bool &r_b_interrupt,
	bool &r_b_write_error, uint64_t &r_n_counter, uint64_t &r_n_filesize,
	CHTTP_File &r_http_file)
	:m_p_s_output(p_s_output), m_r_n_counter(r_n_counter),
	m_r_n_filesize(r_n_filesize),
#ifdef __DOWNLOADER_PREALLOCATE_FILE
	m_p_fw(INVALID_HANDLE_VALUE),
#else //__DOWNLOADER_PREALLOCATE_FILE
	m_p_fw(0),
#endif //__DOWNLOADER_PREALLOCATE_FILE
	m_r_http_file(r_http_file),
	m_r_b_interrupt(r_b_interrupt), m_r_b_write_error(r_b_write_error)
#ifdef __DOWNLOADER_NULL_CHECK
	, m_n_null_check(0)
#endif //__DOWNLOADER_NULL_CHECK
{
	m_r_b_write_error = false;
}

CFileReceiver::~CFileReceiver()
{
	if(m_p_fw)
		fclose(m_p_fw);
	else {
		/*FILE *p_fw;
		if(!(p_fw = ::fopen(m_p_s_output, "wb"))) {
			m_r_b_write_error = true;
			return;
		}
		::fclose(p_fw);*/
		// don't do this! it will always overwrite the file if the compiler chooses
		// to copy the CFileReceiver as it is passed to CHTTP_File::Receive()

		// in case the file wasn't opened at all, do it now
		// (the download may have succeeded with no body, or in contrast failed, while
		// downloading to a file that already existed and we should now erase it's contents)
	}
}

bool CFileReceiver::operator ()(const void *p_data, size_t n_size)
{
	if(m_r_b_interrupt)
		return false;
	// fake interrupt

	if(!m_r_n_filesize) {
		m_r_n_filesize = m_r_http_file.n_Size();
		//printf("file size %d B\n", m_r_n_filesize); // debug
	}
	// set filesize

	m_r_n_counter += n_size;
	//printf("downloaded %d B (total %d B)\n", n_size, m_r_n_counter); // debug
	// count downloaded size

#ifdef __DOWNLOADER_PREALLOCATE_FILE
	if(m_p_fw == INVALID_HANDLE_VALUE) {
		if((m_p_fw = h_OpenPreallocateFile(m_p_s_output, m_r_http_file.n_Size())) == INVALID_HANDLE_VALUE) {
#else //__DOWNLOADER_PREALLOCATE_FILE
	if(!m_p_fw) {
		if(!(m_p_fw = fopen(m_p_s_output, "wb"))) {
#endif //__DOWNLOADER_PREALLOCATE_FILE
			m_r_b_write_error = true;
			return false;
		}
		// open file for writing

#ifdef __DOWNLOADER_FORMAT_CHECK
		const char *p_s_extension = strrchr(m_p_s_output, '.');
		if(p_s_extension) {
			++ p_s_extension;
			// get output file extension

			if(n_size >= 4) {
				if((!strcasecmp(p_s_extension, "rar") || // "rar"
				   (strlen(p_s_extension) == 3 && tolower(p_s_extension[0]) == 'r' &&
				   isdigit((uint8_t)p_s_extension[1]) && isdigit((uint8_t)p_s_extension[2]))) && // "r01", ...
				   memcmp(p_data, "Rar!", 4 * sizeof(char))) {
					// do not set write error, make it look like download error
					return false;
				}
				const uint32_t n_zip_signature = 0x04034b50;
				if(!strcasecmp(p_s_extension, "zip") && memcmp(p_data, &n_zip_signature, sizeof(uint32_t))) {
					// do not set write error, make it look like download error
					return false;
				}
			} else {
				//printf("very small fragment\n"); // debug
			}
			// check if format we're downloading contains valid header (sometimes proxies insert some rubbish)
		}
#endif //__DOWNLOADER_FORMAT_CHECK

#ifdef __DOWNLOADER_NULL_CHECK
		if(n_size >= checked_NullSequence) {
			for(size_t i = 0; /*i < n_size*/; ++ i) {
				_ASSERTE(i < n_size);
				if(((const uint8_t*)p_data)[i] != 0) {
					m_n_null_check = 0;
					break;
				}
				else if(i + 1 == checked_NullSequence)
					return false; // there is too many nulls, it would seem we're getting rubbish
			}
			// check if there's just too many zeros
		} else {
			m_n_null_check = checked_NullSequence - n_size;
			// how much remains to check

			for(size_t i = 0; i < n_size; ++ i) {
				if(((const uint8_t*)p_data)[i] != 0) {
					m_n_null_check = 0;
					break;
				}
			}
			// check if there's just too many zeros
		}
		// avoid long sequences of zeros (occurs on rapidshare.com sometimes)
#endif //__DOWNLOADER_NULL_CHECK
	}
	// open file for writing, check format checkers

#ifdef __DOWNLOADER_NULL_CHECK
	if(m_n_null_check > 0 && n_size > 0) {
		size_t n_remains = min(n_size, m_n_null_check);
		for(size_t i = 0; i < n_remains; ++ i) {
			_ASSERTE(i < n_size);
			if(((const uint8_t*)p_data)[i] != 0) {
				m_n_null_check = 0;
				break;
			}
		}
		// check if there's just too many zeros

		if(m_n_null_check) {
			m_n_null_check -= n_remains;
			if(!m_n_null_check)
				return false; // there is too many nulls, it would seem we're getting rubbish
		}
		_ASSERTE(m_n_null_check >= 0);
		// see if we checked enough already
	}
	// avoid long sequences of zeros (occurs on rapidshare.com sometimes)
#endif //__DOWNLOADER_NULL_CHECK

	if(fwrite(p_data, n_size, 1, m_p_fw) != 1) {
		m_r_b_write_error = true;
		return false;
	}
	return true;
}

/*
 *								=== ~CFileReceiver ===
 */

/*
 *								=== CHTTPDownloader ===
 */

CHTTPDownloader::CHTTPDownloader()
	:m_n_proxy_retry_num(1), m_n_timeout(-1), m_b_use_post(false)
{}

int CHTTPDownloader::n_Download(const char *p_s_filename, const char *p_s_url, int n_tag)
{
	Reset_DownloadStatus();

	m_n_tag = n_tag;
	// set downloader parameters

	if(!stl_ut::AssignCStr(m_s_filename, p_s_filename) ||
	   !stl_ut::AssignCStr(m_s_url, p_s_url))
		return down_LowMemoryFailure;
	// copy the strings

	return n_Download_FILE();
}

void CHTTPDownloader::Set_Timeout(int n_timeout_seconds)
{
	m_n_timeout = n_timeout_seconds;
}

int CHTTPDownloader::n_Timeount() const
{
	return m_n_timeout;
}

void CHTTPDownloader::Set_RequestType(bool b_use_POST)
{
	m_b_use_post = b_use_POST;
}

bool CHTTPDownloader::b_Use_GET() const
{
	return !m_b_use_post;
}

bool CHTTPDownloader::b_Use_POST() const
{
	return m_b_use_post;
}

void CHTTPDownloader::Set_NoProxy()
{
	m_s_proxy.erase();
}

bool CHTTPDownloader::Set_Proxy(const char *p_s_proxy, int n_retry_num)
{
	_ASSERTE(p_s_proxy && strlen(p_s_proxy)); // use SetNoProxy() to disable proxy
	m_n_proxy_retry_num = n_retry_num;
	return stl_ut::AssignCStr(m_s_proxy, p_s_proxy);
}

const char *CHTTPDownloader::p_s_Proxy() const
{
	if(m_s_proxy.empty())
		return 0;
	return m_s_proxy.c_str();
}

int CHTTPDownloader::n_Proxy_Retry_Num() const
{
	return m_n_proxy_retry_num;
}

bool CHTTPDownloader::Set_RequestParams(const char *p_s_extra_get_params)
{
	if(!p_s_extra_get_params || !*p_s_extra_get_params) {
		m_s_extra_get_params.erase();
		return true;
	}
	return stl_ut::AssignCStr(m_s_extra_get_params, p_s_extra_get_params);
}

const char *CHTTPDownloader::p_s_RequestParameters()
{
	return m_s_extra_get_params.c_str();
}

const char *CHTTPDownloader::p_s_FileName() const
{
	return m_s_filename.c_str();
}

const std::string &CHTTPDownloader::r_s_FileName() const
{
	return m_s_filename;
}

const char *CHTTPDownloader::p_s_URL() const
{
	return m_s_url.c_str();
}

const std::string &CHTTPDownloader::r_s_URL() const
{
	return m_s_url;
}

uint64_t CHTTPDownloader::n_File_Size() const
{
	return m_n_file_size;
}

uint64_t CHTTPDownloader::n_Downloaded_Size() const
{
	return m_n_downloaded;
}

int CHTTPDownloader::n_HTTP_ResponseCode() const
{
	return m_n_http_status;
}

const CHTTP_Header &CHTTPDownloader::r_HTTP_Header() const
{
	return m_http_header;
}

int CHTTPDownloader::n_Tag() const
{
	return m_n_tag;
}

void CHTTPDownloader::Reset_DownloadStatus()
{
	m_b_write_err = false;
	m_n_file_size = 0;
	m_n_downloaded = 0;
	m_n_http_status = 0;
	m_b_interrupt = false;
}

int CHTTPDownloader::n_Download_FILE()
{
	CHTTP_File http_file(m_s_url.c_str());
	if(!http_file.p_s_Location())
		return down_LowMemoryFailure;
	// prepare HTTP file

	bool b_result;
	const char *p_s_get_params = (m_s_extra_get_params.empty())? 0 : m_s_extra_get_params.c_str();
	if(!m_s_proxy.empty()) {
		for(int n_retry = 0; n_retry < m_n_proxy_retry_num; ++ n_retry) {
			b_result = http_file.ProxiedReceive(m_s_proxy.c_str(), CFileReceiver(m_s_filename.c_str(),
			   m_b_interrupt, m_b_write_err, m_n_downloaded, m_n_file_size, http_file),
			   p_s_get_params, 5, 4096, m_b_use_post, m_n_timeout);
			if(b_result)
				break;
		}
	} else {
		b_result = http_file.Receive(CFileReceiver(m_s_filename.c_str(), m_b_interrupt,
			m_b_write_err, m_n_downloaded, m_n_file_size, http_file), p_s_get_params,
			5, 4096, m_b_use_post, m_n_timeout);
	}
	// let itself redirect

	m_n_http_status = (http_file.p_Header())? http_file.p_Header()->n_Status_Code() : -1;
	// read http status

	try {
		if(http_file.p_Header())
			m_http_header = *http_file.p_Header();
	} catch(std::bad_alloc&) {
		return down_LowMemoryFailure;
	}
	// copy HTTP response header

	if(!stl_ut::AssignCStr(m_s_url, http_file.p_s_Location()))
		return down_LowMemoryFailure;
	// remember the final url (may redirect)

	if(m_b_interrupt)
		return down_Interrupted;
	// can't happen, this is not threaded

	if(m_b_write_err)
		return down_WriteError;

	_ASSERTE(http_file.p_Header() || m_n_http_status < 0);
	if(!b_result || m_n_http_status < 200 || m_n_http_status >= 400)
		return down_Failure;
	// it should end with http 300, 200 would be ok too

	if(m_n_downloaded == 0) {
		FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, m_s_filename.c_str(), "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if((p_fw = fopen(m_s_filename.c_str(), "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			fclose(p_fw);
		else
			return down_WriteError;
	}
	// in case the file is zero-size, we need to create an empty file

	return down_Success;
}

/*
 *								=== ~CHTTPDownloader ===
 */

#ifndef __NO_THREADED_DOWNLOADER

/*
 *								=== CThreadedHTTPDownloader ===
 */

CThreadedHTTPDownloader::CThreadedHTTPDownloader()
	:CHTTPDownloader(), m_assignment_sema(1),
	m_b_assignment(false), m_n_result(down_Idle)
{
	m_thread.AttachRunable(*this);
}

bool CThreadedHTTPDownloader::b_IsAssigned() const
{
	return m_b_assignment;
}

bool CThreadedHTTPDownloader::b_IsRunning() const
{
	return m_n_result == down_Running;
}

int CThreadedHTTPDownloader::n_Download(const char *p_s_filename,
	const char *p_s_url, int n_tag, bool *p_b_was_assigned)
{
	if(p_b_was_assigned)
		*p_b_was_assigned = false;
	// failures are failures

	{
		if(!m_assignment_sema.TryWait())
			return down_Failure;
		// enter only if not running right now

		if(m_b_assignment) {
			m_assignment_sema.Signal();

			if(p_b_was_assigned)
				*p_b_was_assigned = true;
			// failure mean was assigned

			return down_Failure;
		}
		// it was not running, but the original caller didn't call n_GetResult() yet

		m_b_assignment = true;
		// mark as assigned
	}
	// handle assignments

	Reset_DownloadStatus();

	m_n_tag = n_tag;
	// set downloader parameters

	if(!stl_ut::AssignCStr(m_s_filename, p_s_filename) ||
	   !stl_ut::AssignCStr(m_s_url, p_s_url)) {
		m_assignment_sema.Signal(); // ...
		return down_LowMemoryFailure;
	}
	// copy the strings

	m_n_result = down_Running;

	int n_result = n_Download_FILE();

	m_n_result = down_Idle;

	m_assignment_sema.Signal();
	// free the semaphore, but don't mark as unassigned

	return n_result;
}

bool CThreadedHTTPDownloader::Start_Download(const char *p_s_filename,
	const char *p_s_url, int n_tag, bool *p_b_was_assigned)
{
	if(p_b_was_assigned)
		*p_b_was_assigned = false;
	// failures are failures

	{
		if(!m_assignment_sema.TryWait())
			return down_Failure;
		// enter only if not running right now

		if(m_b_assignment) {
			m_assignment_sema.Signal();

			if(p_b_was_assigned)
				*p_b_was_assigned = true;
			// failure mean was assigned

			return down_Failure;
		}
		// it was not running, but the original caller didn't call n_GetResult() yet

		m_b_assignment = true;
		// mark as assigned
	}
	// handle assignments

	Reset_DownloadStatus();

	m_n_tag = n_tag;
	// set downloader parameters

	if(!stl_ut::AssignCStr(m_s_filename, p_s_filename) ||
	   !stl_ut::AssignCStr(m_s_url, p_s_url)) {
		m_assignment_sema.Signal(); // ...
		return false;
	}
	// copy the strings

	if(!m_thread.Start()) {
		m_assignment_sema.Signal(); // ...
		return false;
	}
	// if the thread fails to start, clear the assignment flag

	m_n_result = down_Running; // !!

	return true;
}

int CThreadedHTTPDownloader::n_Stop(bool b_wait_for_thread)
{
	m_b_interrupt = true;

	if(b_wait_for_thread) {
		if(!WaitForFinish())
			return down_Failure;
	}

	return m_n_result;
	// if b_wait_for_thread is set, it is guaranteed to be the correct final result
}

bool CThreadedHTTPDownloader::WaitForFinish()
{
	if(!m_assignment_sema.Wait())
		return false; // wait for the semaphore failed ... something is wrong

	m_thread.Stop(false);
	// make sure the thread stopped  as well (the thread may Signal() the semaphore
	// and then yield to this thread, therefore it's still running). this is important
	// if the caller wants to call Start_Download() after this function returns.

	_ASSERTE(!m_thread.b_IsRunning()); // the thread should not be running now
	m_assignment_sema.Signal(); // we entered the semaphore, let's free it
	return true;
}

int CThreadedHTTPDownloader::n_PeekResult() const
{
	return m_n_result;
}

int CThreadedHTTPDownloader::n_GetResult(bool b_wait_for_finish_if_running)
{
	if(b_wait_for_finish_if_running) {
		if(!m_assignment_sema.Wait())
			return down_Failure; // semaphore wait failed
	} else {
		if(!m_assignment_sema.TryWait())
			return down_Running;
	}
	// make sure it's not running (wait or return)

	m_thread.Stop(false);
	// make sure the thread stopped  as well (the thread may Signal() the semaphore
	// and then yield to this thread, therefore it's still running). this is important
	// if the caller wants to call Start_Download() after this function returns.

	_ASSERTE(m_b_assignment || m_n_result == down_Idle);
	// if it was not assigned, the result should be down_Idle

	int n_result = m_n_result;
	// finished; remember the result

	m_n_result = down_Idle;
	// clear the assignment flag

	m_b_assignment = false;
	m_assignment_sema.Signal();
	// not assigned

	return n_result;
	// return the saved result (the downloader may already be in use by a different thread)
}

void CThreadedHTTPDownloader::Run()
{
	//_ASSERTE(m_n_result == down_Running);
	// it's assigned *after* the thread is launched so this is probably not a wise thing to do

	m_n_result = down_Running; // not neccessary, it's probably already there
	m_n_result = n_Download_FILE();
	m_assignment_sema.Signal(); // not running anymore
}

/*
 *								=== ~CThreadedHTTPDownloader ===
 */

#endif // !__NO_THREADED_DOWNLOADER
