/*
								+--------------------------------+
								|                                |
								|   ***  FFmpeg interface  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2008  |
								|                                |
								|          PNGLoad.cpp           |
								|                                |
								+--------------------------------+
*/

/**
 *	@file iface/FFmpeg.cpp
 *	@date 2008
 *	@author -tHE SWINe-
 *	@brief FFmpeg interface
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include "FFmpeg.h"
extern "C" {
#include <ffmpeg/avformat.h>
#include <ffmpeg/avcodec.h>
}

#if defined(_MSC_VER) && !defined(__MWERKS__)
#pragma comment(lib, "avcodec-51.lib")
#pragma comment(lib, "avformat-51.lib")
#pragma comment(lib, "avutil-49.lib")
#pragma comment(lib, "swscale-0.lib")
#endif // _MSC_VER && !__MWERKS__
// link instructions (windows only)

/**
 *	@brief initializes libavformat and register all the muxers, demuxers and protocols
 */
class CLibAVInitializer {
public:
	/**
	 *	@brief default constructor
	 */
	CLibAVInitializer()
	{
		av_register_all();
		// initialize libavformat and register all the muxers, demuxers and protocols
	}
};

static CLibAVInitializer libav_init; /**< @brief libav initializer */

/*
 *								=== CFFmpegVideo ===
 */

CFFmpegVideo::CFFmpegVideo()
	:m_p_format_context(0), m_p_codec(0), m_p_codec_context(0), m_p_frame(0),
	m_p_frame_RGB(0), m_n_video_stream_id(size_t(-1)), m_p_RGB_frame_data(0)
{}

CFFmpegVideo::CFFmpegVideo(const char *p_s_filename,
	bool b_dump_format /*= true*/, size_t n_video_stream_index /*= 0*/)
	:m_p_format_context(0), m_p_codec(0), m_p_codec_context(0), m_p_frame(0),
	m_p_frame_RGB(0), m_n_video_stream_id(size_t(-1)), m_p_RGB_frame_data(0)
{
	Open(p_s_filename, b_dump_format, n_video_stream_index);
}

CFFmpegVideo::~CFFmpegVideo()
{
	Close();
}

bool CFFmpegVideo::Open(const char *p_s_filename,
	bool b_dump_format /*= true*/, size_t n_video_stream_index /*= 0*/)
{
	Close();
	// !!

	m_f_play_time = 0;
	// start playing from the beginning

	if(av_open_input_file(&m_p_format_context, p_s_filename, NULL, 0, NULL) != 0)
		return false;
	do {
		if(av_find_stream_info(m_p_format_context) < 0)
			break;
		if(b_dump_format)
			dump_format(m_p_format_context, 0, p_s_filename, 0); // to stderr
		// open video file

		m_n_video_stream_id = size_t(-1);
		for(size_t i = 0; i < m_p_format_context->nb_streams; ++ i) {
			if(m_p_format_context->streams[i]->codec->codec_type == CODEC_TYPE_VIDEO) {
				if(!n_video_stream_index) {
					m_n_video_stream_id = i;
					break;
				}
				-- n_video_stream_index;
			}
		}
		if(m_n_video_stream_id == size_t(-1))
			break;
		// find the first video stream

		m_p_codec_context = m_p_format_context->streams[m_n_video_stream_id]->codec;
		// get a pointer to the codec context for the video stream

		if((m_p_codec = avcodec_find_decoder(m_p_codec_context->codec_id)) == NULL) {
			if(b_dump_format)
				fprintf(stderr, "error: unsupported codec\n");
			break; // codec not found
		}
		// find the decoder for the video stream

		if(avcodec_open(m_p_codec_context, m_p_codec) < 0) {
			if(b_dump_format)
				fprintf(stderr, "error: codec failed to open\n");
			break; // could not open codec
		}
		// open codec

		do {
			/*videoWidth = m_p_codec_context->width;
			videoHeight = m_p_codec_context->height;*/ // not needed to store this twice
			if(m_p_format_context->streams[m_n_video_stream_id]->r_frame_rate.den &&
			   m_p_format_context->streams[m_n_video_stream_id]->r_frame_rate.num)
				m_f_frame_time = 1 / av_q2d(m_p_format_context->streams[m_n_video_stream_id]->r_frame_rate);
			else
				m_f_frame_time = av_q2d(m_p_codec_context->time_base);
			m_f_duration = m_p_format_context->duration / double(AV_TIME_BASE);
			// get video parameters

			if(!(m_p_frame = avcodec_alloc_frame()))
				break;
			do {
				if(!(m_p_frame_RGB = avcodec_alloc_frame()))
					break;
				do {
					size_t n_RGB_frame_size = avpicture_get_size(PIX_FMT_RGB32,
						m_p_codec_context->width, m_p_codec_context->height);
					if(!(m_p_RGB_frame_data = (uint8_t*)av_malloc(n_RGB_frame_size * sizeof(uint8_t))))
						break;
					// determine required m_p_RGB_frame_data size and allocate m_p_RGB_frame_data for the decoded picture

					avpicture_fill((AVPicture*)m_p_frame_RGB, m_p_RGB_frame_data, PIX_FMT_RGB32,
						m_p_codec_context->width, m_p_codec_context->height);
					// assign appropriate parts of m_p_RGB_frame_data to image planes in m_p_frame_RGB
					// (note that m_p_frame_RGB is an AVFrame, but AVFrame is a superset
					// of AVPicture)

					m_t_frame.b_alpha = m_p_codec_context->pix_fmt == PIX_FMT_RGB555 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB32 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR32 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR555 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB32_1 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR32_1;
					// alpha

					m_t_frame.b_grayscale = m_p_codec_context->pix_fmt == PIX_FMT_GRAY8 ||
						m_p_codec_context->pix_fmt == PIX_FMT_MONOWHITE ||
						m_p_codec_context->pix_fmt == PIX_FMT_MONOBLACK ||
						m_p_codec_context->pix_fmt == PIX_FMT_GRAY16LE ||
						m_p_codec_context->pix_fmt == PIX_FMT_GRAY16BE;
					// grayscale

					m_t_frame.n_former_bpc = (m_p_codec_context->pix_fmt == PIX_FMT_MONOWHITE ||
						m_p_codec_context->pix_fmt == PIX_FMT_MONOBLACK)? 1 :
						(m_p_codec_context->pix_fmt == PIX_FMT_GRAY16LE ||
						m_p_codec_context->pix_fmt == PIX_FMT_GRAY16BE)? 16 :
						(m_p_codec_context->pix_fmt == PIX_FMT_BGR4 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR4_BYTE)? 2 :
						(m_p_codec_context->pix_fmt == PIX_FMT_BGR8 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB8 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB4 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB4_BYTE)? 3 :
						(m_p_codec_context->pix_fmt == PIX_FMT_BGR32 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR565 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR555 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB565 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB555)? 5 :
						(m_p_codec_context->pix_fmt == PIX_FMT_BGR32_1 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB32_1 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR32 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB32 ||
						m_p_codec_context->pix_fmt == PIX_FMT_RGB24 ||
						m_p_codec_context->pix_fmt == PIX_FMT_BGR24)? 8 : 4;
					// approximate bits per channel

					m_t_frame.n_width = m_p_codec_context->width;
					m_t_frame.n_height = m_p_codec_context->height;
					// dimensions

					_ASSERTE(m_p_frame_RGB->linesize[0] == m_t_frame.n_width * sizeof(uint32_t) &&
						!m_p_frame_RGB->linesize[1] && !m_p_frame_RGB->linesize[2] && !m_p_frame_RGB->linesize[3]);
					_ASSERTE(!m_p_frame_RGB->data[1] && !m_p_frame_RGB->data[2] && !m_p_frame_RGB->data[3]);
					// make sure the scanlines are tight and the channels are interleaved

					m_t_frame.p_buffer = (uint32_t*)m_p_frame_RGB->data[0]; // this might not work with big endian
					// setup the image

					return true;
					// success

				} while(0);
				av_free(m_p_frame_RGB);
				m_p_frame_RGB = 0;
			} while(0);
			av_free(m_p_frame);
			m_p_frame = 0;
			// alloc frames (one for the source format (YUV) and one for RGB)

		} while(0);
		avcodec_close(m_p_codec_context);
		m_p_codec_context = 0;
		// init m_p_codec_context

	} while(0);
	av_close_input_file(m_p_format_context);
	m_p_format_context = 0;
	// init m_p_format_context

	return false;
	// failure
}

void CFFmpegVideo::Close()
{
	if(!m_p_RGB_frame_data) {
		_ASSERTE(!m_p_frame);
		_ASSERTE(!m_p_frame_RGB);
		_ASSERTE(!m_p_codec);
		_ASSERTE(!m_p_codec_context);
		_ASSERTE(!m_p_format_context);
		// nothing else should be allocated either
		return;
	}
	// everything is allocated

	av_free(m_p_RGB_frame_data);
	av_free(m_p_frame_RGB);
	m_p_RGB_frame_data = 0;
	m_p_frame_RGB = 0;
	// free the RGB image

	av_free(m_p_frame);
	m_p_frame = 0;
	// free the YUV frame

	avcodec_close(m_p_codec_context);
	m_p_codec_context = 0;
	// close the codec

	av_close_input_file(m_p_format_context);
	m_p_format_context = 0;
	// close the video file

	m_p_codec = 0;
	// not deleted (allocated and managed by FFmpeg), just set it to 0

	TBmp noimg = {0};
	m_t_frame = noimg;
	// clear frame to avoid accidentally pointing to a bad memory

	m_f_frame_time = 0;
	m_f_duration = 0;
	m_f_play_time = 0;
	// clear time info
}

bool CFFmpegVideo::Seek(double f_seek_time, bool b_relative /*= false*/)
{
	if(!m_p_RGB_frame_data)
		return false;
	// make sure a video is opened

	double targetTime = (b_relative)? m_f_play_time + f_seek_time : f_seek_time;
	double deltaTime = targetTime - m_f_play_time;
	// determine target time and delta time

	int64_t targetTimeFrames = int64_t(targetTime * AV_TIME_BASE);
	int flags = 0;//(deltaTime < 0)? AVSEEK_FLAG_BACKWARD : 0;
	// convert from seconds to time frames (not images - frames)

	const AVRational tbq = {1, AV_TIME_BASE};
	targetTimeFrames = av_rescale_q(targetTimeFrames, tbq,
		m_p_format_context->streams[m_n_video_stream_id]->time_base);
	// convert time frames ... somehow ...

	if(av_seek_frame(m_p_format_context, m_n_video_stream_id, targetTimeFrames, flags) >= 0) {
		m_f_play_time = targetTime;
		return true;
	}
	return false;
}

const TBmp &CFFmpegVideo::t_Get_NextFrame()
{
	static TBmp noimg = {0};
	if(!m_p_RGB_frame_data)
		return noimg; // no frames
	// make sure a video is opened

	int frameFinished;
	AVPacket packet;
	while(av_read_frame(m_p_format_context, &packet) >= 0) {
		// Is this a packet from the video stream?
		if(packet.stream_index == m_n_video_stream_id) {
			avcodec_decode_video(m_p_codec_context, m_p_frame, &frameFinished,
				packet.data, packet.size);
			// decode video frame

			if(frameFinished) {
				img_convert((AVPicture*)m_p_frame_RGB, PIX_FMT_RGB32, 
					(const AVPicture*)m_p_frame, m_p_codec_context->pix_fmt, 
					m_p_codec_context->width, m_p_codec_context->height);
				// convert the image from its native format to RGB

				m_f_play_time += m_f_frame_time;
				// maintain m_f_play_time

				av_free_packet(&packet);
				// free the packet that was allocated by av_read_frame

				return m_t_frame;
				// return the prepared image structure (the pointer inside points to m_p_frame_RGB data)
			}
			// did we get a video frame?
		}

		// Free the packet that was allocated by av_read_frame
		av_free_packet(&packet);
	}

	return noimg; // no frames
}

/*
 *								=== ~CFFmpegVideo ===
 */
