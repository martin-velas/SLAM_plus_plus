/*
								+--------------------------------+
								|                                |
								|   ***  FFmpeg interface  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|           PNGLoad.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __FFMPEG_WRAPPER_INCLUDED
#define __FFMPEG_WRAPPER_INCLUDED

/**
 *	@file iface/FFmpeg.h
 *	@date 2008
 *	@author -tHE SWINe-
 *	@brief FFmpeg interface
 */

#include "../Bitmap.h"

struct AVFormatContext;
struct AVCodec;
struct AVCodecContext;
struct AVFrame;
// forward declarations, in case the user does not want to include FFmpeg

/**
 *	@brief a simple FFmpeg video wrapper
 *
 *	This decodes video frames as BGRA8, and ignores any audio or other streams.
 */
class CFFmpegVideo {
protected:
	AVFormatContext *m_p_format_context;
	AVCodec *m_p_codec;
	AVCodecContext *m_p_codec_context;
	AVFrame *m_p_frame, *m_p_frame_RGB;
	size_t m_n_video_stream_id;
	uint8_t *m_p_RGB_frame_data;
	// FFmpeg state

	TBmp m_t_frame;
	// returned frame

	double m_f_frame_time; // in seconds
	double m_f_duration; // in seconds
	double m_f_play_time; // in seconds
	// video properties

public:
	/**
	 *	@brief default constructor
	 */
	CFFmpegVideo();

	/**
	 *	@brief constructor; opens a video file
	 *
	 *	@param[in] p_s_filename is null-terminated string, containing video file name
	 *	@param[in] b_dump_format is format dump flag (if set. FFmpeg outputs some information to stderr)
	 *	@param[in] n_video_stream_index is relative video stream index (0 being the first video stream
	 *		in the file, but not neccessarily the stream 0)
	 *
	 *	@note This function might fail, call b_Opened() to see if the video was indeed opened.
	 */
	CFFmpegVideo(const char *p_s_filename, bool b_dump_format = true, size_t n_video_stream_index = 0);

	/**
	 *	@brief destructor; closes video file if any is opened
	 *	@note Any frames returned by t_Get_NextFrame() are invalidated.
	 */
	~CFFmpegVideo();

	/**
	 *	@brief opens a video file
	 *
	 *	@param[in] p_s_filename is null-terminated string, containing video file name
	 *	@param[in] b_dump_format is format dump flag (if set. FFmpeg outputs some information to stderr)
	 *	@param[in] n_video_stream_index is relative video stream index (0 being the first video stream
	 *		in the file, but not neccessarily the stream 0)
	 *
	 *	@return Returns true on success, false on failure.
	 */
	bool Open(const char *p_s_filename, bool b_dump_format = true, size_t n_video_stream_index = 0);

	inline bool b_Opened() const
	{
		return m_p_RGB_frame_data != 0;
	}

	inline const AVFormatContext *p_FormatContext() const
	{
		return m_p_format_context;
	}

	inline const AVCodec *p_Codec() const
	{
		return m_p_codec;
	}

	inline const AVCodecContext *p_CodecContext() const
	{
		return m_p_codec_context;
	}

	inline int n_Width() const
	{
		return (m_p_codec_context)? m_t_frame.n_width : -1;
	}

	inline int n_Height() const
	{
		return (m_p_codec_context)? m_t_frame.n_height : -1;
	}

	inline double f_FrameTime() const
	{
		return m_f_frame_time;
	}

	inline double f_FPS() const
	{
		return 1 / m_f_frame_time;
	}

	inline double f_Duration() const
	{
		return m_f_duration;
	}

	inline double f_NextFrame_Time() const
	{
		return m_f_play_time;
	}

	void Close();

	bool Seek(double f_seek_time, bool b_relative = false);

	const TBmp &t_Get_NextFrame();

private:
	CFFmpegVideo(const CFFmpegVideo &r_other); // no-copy
	CFFmpegVideo &operator =(const CFFmpegVideo &r_other); // no-copy
};

#endif // !__FFMPEG_WRAPPER_INCLUDED
