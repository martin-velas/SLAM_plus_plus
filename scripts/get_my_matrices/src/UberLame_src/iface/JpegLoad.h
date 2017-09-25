/*
								+---------------------------------+
								|                                 |
								|   ***  jpeglib interface  ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2005  |
								|                                 |
								|           JpegLoad.h            |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __JPEGLIB_WRAPPER_INCLUDED
#define __JPEGLIB_WRAPPER_INCLUDED

/**
 *	@file iface/JpegLoad.h
 *	@date 2005
 *	@author -tHE SWINe-
 *	@brief jpeglib interface
 */

#include "../Bitmap.h"

/**
 *	@brief JPEG image format coder / decoder
 */
class CJpegCodec {
public:
	/**
	 *	@brief JPEG colorspace names
	 */
	enum {
		jcolor_Gray = 1, /**< grayscale colorspace */
		jcolor_YCbCr = 3, /**< YCbCr (3-component color) colorspace */
		jcolor_YCCK = 4 /**< YCCK (4-component color) colorspace */
	};

public:
	/**
	 *	@brief loads jpeg file
	 *
	 *	param[in] p_s_filename is input file name
	 *
	 *	@return Returns pointer to loaded bitmap on success, or 0 on failure.
	 */
	static TBmp *p_Load_JPEG(const char *p_s_filename);

	/**
	 *	@brief saves bitmap in memory to jpeg file
	 *
	 *	param[in] p_s_filename is output file name
	 *	param[in] r_bmp is reference to the bitmap to save as jpeg file
	 *	param[in] n_colorspace is output colorspace (one of jcolor_Gray, jcolor_YCbCr or jcolor_YCCK)
	 *	param[in] n_quality is output quality, represented by integer in range 0 to 100
	 *	param[in] b_optimize enables Huffman code optimization
	 *	param[in] n_subsampling_x is first component subsampling in horizontal direction
	 *	param[in] n_subsampling_y is first component subsampling in vertical direction
	 *	param[in] b_allow_16bit_quant_tables enables having 16-bit quantization tables
	 *	param[in] b_progressive enables writing output as progressive jpeg
	 *	param[in] n_restart_interval is restart interval; 0 = disabled, values 1 to 65535 = enabled
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Save_JPEG(const char *p_s_filename, const TBmp &r_bmp, bool b_BGRA = false,
		int n_colorspace = jcolor_YCbCr, int n_quality = 90, bool b_optimize = true,
		int n_subsampling_x = 1, int n_subsampling_y = 1, bool b_allow_16bit_quant_tables = false,
		bool b_progressive = false, int n_restart_interval = 0);
};

#endif // !__JPEGLIB_WRAPPER_INCLUDED
