/*
								+--------------------------------+
								|                                |
								|   ***  libpng interface  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2005  |
								|                                |
								|           PNGLoad.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __LIBPNG_WRAPPER_INCLUDED
#define __LIBPNG_WRAPPER_INCLUDED

/**
 *	@file iface/PNGLoad.h
 *	@date 2005
 *	@author -tHE SWINe-
 *	@brief libpng interface
 */

#include "../Bitmap.h"

/**
 *	@brief PNG image format coder / decoder
 */
class CPngCodec {
public:
	/**
	 *	@brief information about a PNG image
	 */
	struct TImageInfo {
		bool b_alpha; /**< @brief alpha channel flag */
		bool b_grayscale; /**< @brief grayscale flag */
		int n_bit_depth; /**< @brief number of bits per pixel (or per palette color) */
		int n_palette_entry_num; /**< @brief number of palette entries or 0 for true color */
		int n_width; /**< @brief image width */
		int n_height; /**< @brief image height */
	};

protected:
	/*static TBmp *p_Load_PNG_int(png_structp p_png_ptr);
	static bool GetInfo_int(TImageInfo &r_t_info, png_structp p_png_ptr);*/
	class CInternal;

public:
	/**
	 *	@brief reads image header from a file and returns image properties
	 *
	 *	@param[out] r_t_info is structure to be filled with image information
	 *	@param[in] p_s_filename is input file name
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_ImageInfo(TImageInfo &r_t_info, const char *p_s_filename);

	/**
	 *	@brief reads image header from memory and returns image properties
	 *
	 *	@param[out] r_t_info is structure to be filled with image information
	 *	@param[in] p_data is image data
	 *	@param[in] n_size is image data size, in bytes
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_ImageInfo(TImageInfo &r_t_info, const void *p_data, size_t n_size);

	/**
	 *	@brief loads jpeg image from file
	 *
	 *	@param[in] p_s_filename is input file name
	 *
	 *	@return Returns pointer to loaded bitmap on success, or 0 on failure.
	 */
	static TBmp *p_Load_PNG(const char *p_s_filename);

	/**
	 *	@brief loads jpeg image from memory
	 *
	 *	@param[in] p_data is image data
	 *	@param[in] n_size is image data size, in bytes
	 *
	 *	@return Returns pointer to loaded bitmap on success, or 0 on failure.
	 */
	static TBmp *p_Load_PNG(const void *p_data, size_t n_size);

	/**
	 *	@brief saves an image to a file
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] r_t_bmp is the image to be saved (if r_t_bmp.b_alpha is set,
	 *		the image is written as RGBA, otherwise as RGB. also, if r_t_bmp.b_grayscale
	 *		is set (while r_t_bmp.b_alpha is not), the image is written
	 *		as RLE - compressed grayscale).
	 *	@param[in] b_BGRA is set if the image is BGRA (images loaded by p_Load_TGA() are BGRA)
	 *	@param[in] b_interlace is png interlace flag (not used for grayscale / grayscale alpha)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note The t_bmp.n_former_bpc field is ignored, images are always saved as 8bpp.
	 */
	static bool Save_PNG(const char *p_s_filename, const TBmp &r_t_bitmap,
		bool b_BGRA = false, bool b_interlace = false);

	/**
	 *	@brief saves an image to a memory buffer
	 *
	 *	@param[out] r_dest_buffer is buffer, filled with compressed image data
	 *	@param[in] r_t_bmp is the image to be saved (if r_t_bmp.b_alpha is set,
	 *		the image is written as RGBA, otherwise as RGB. also, if r_t_bmp.b_grayscale
	 *		is set (while r_t_bmp.b_alpha is not), the image is written
	 *		as RLE - compressed grayscale).
	 *	@param[in] b_BGRA is set if the image is BGRA (images loaded by p_Load_TGA() are BGRA)
	 *	@param[in] b_interlace is png interlace flag (not used for grayscale / grayscale alpha)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note The t_bmp.n_former_bpc field is ignored, images are always saved as 8bpp.
	 */
	static bool Save_PNG(std::vector<uint8_t> &r_dest_buffer, const TBmp &r_t_bitmap,
		bool b_BGRA = false, bool b_interlace = false);
};

#endif // !__LIBPNG_WRAPPER_INCLUDED
