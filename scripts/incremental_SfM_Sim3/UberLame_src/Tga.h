/*
								+---------------------------------+
								|                                 |
								|        ***   Targa   ***        |
								|                                 |
								|  Copyright  © -tHE SWINe- 2002  |
								|                                 |
								|              Tga.h              |
								|                                 |
								+---------------------------------+
*/

#pragma once
#ifndef __TARGA_INCLUDED
#define __TARGA_INCLUDED

/**
 *	@file Tga.h
 *	@brief Targa bitmap image format support.
 *	@date 2006
 *	@author -tHE SWINe-
 *
 *	@date 2006-06-08
 *
 *	passed code revision
 *	rewrote to C++, used templates and template functions to reduce code length
 *
 *	@date 2006-07-31
 *
 *	added slight template hack to be compilable under g++/linux
 *
 *	@date 2007-03-26
 *
 *	removed template hack, fixed the code so there's same codepath for both g++/linux and msvc/win
 *
 *	@date 2007-12-24
 *
 *	improved linux compatibility by using posix integer types
 *
 *	@date 2008-03-04
 *
 *	now using Integer.h header, created CTgaCodec class, exposing color conversion routines,
 *	added CTgaCodec::Save_TGA() for writing TGA images
 *
 *	@date 2008-05-08
 *
 *	fixed minor issues in CTgaCodec::Save_TGA (RGB / BGR, alpha channel)
 *
 *	@date 2008-11-09
 *
 *	fixed bug in CTgaCodec::Save_TGA in greyscale RLE code, runs shorter than 3 pixels came
 *	undetected, there was also typo in RGB / grey decission (= instead of ==)
 *
 *	@date 2009-05-04
 *
 *	fixed mixed windows / linux line endings
 *
 *	@date 2009-10-20
 *
 *	fixed some warnings when compiling under VC 2005, implemented "Security
 *	Enhancements in the CRT" for VC 2008. compare against MyProjects_2009-10-19_
 *
 *	@date 2011-06-17
 *
 *	Improved TBmp to support a simple software rasterization of point, line
 *	and triangle primitives, and some other operations.
 *
 *	@date 2012-01-25
 *
 *	Added the b_BGRA parameter to CTgaCodec::Save_TGA(). This is a shame, but the old
 *	code actually expected different component order than CTgaCodec::p_Load_TGA(). So loading
 *	an image and saving it right away would result in a file with R and B components swapped.
 *	Some software already solved this using some custom BGRA-RGBA loop before saving
 *	so the b_BGRA parameter does not have a default value (which should be true) set in order
 *	to keep the old software working.
 *
 *	@date 2012-02-16
 *
 *	Added the CTgaCodec::Get_ImageInfo() function.
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 */

#include "Bitmap.h"

/**
 *	@brief TGA image format coder / decoder
 */
class CTgaCodec {
public:
	/**
	 *	@brief conversion from indexed color to RGBA8
	 *	@tparam TPaletteIndex is palette index data type
	 */
	template <class TPaletteIndex>
	struct TPaletteColor {
		typedef TPaletteIndex TDataType; /**< @brief palette index data type */

	protected:
		const uint32_t *m_p_palette; /**< @brief palette (RGBA8) */
		int m_n_palette_entries; /**< @brief number of palette entries */
		int m_n_offset; /**< @brief offset into palette */

	public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] p_palette is pointer to the color table (RGBA8)
		 *	@param[in] n_palette_entries is number of palette entries
		 *	@param[in] n_offset is offset into palette
		 */
		TPaletteColor(const uint32_t *p_palette, int n_palette_entries, int n_offset)
			:m_p_palette(p_palette), m_n_palette_entries(n_palette_entries), m_n_offset(n_offset)
		{}

		/**
		 *	@brief performs conversion from indexed color to RGBA8
		 *	@param[in] r_t_color is zero-based palette index
		 *	@return Returns palette[r_t_color + palette offset], or 0 in case the index is out of bounds.
		 */
		inline uint32_t operator()(const TPaletteIndex &r_t_color) const
		{
			return (r_t_color + m_n_offset >= 0 &&
				r_t_color + m_n_offset < m_n_palette_entries)? m_p_palette[r_t_color + m_n_offset] : 0;
		}
	};

	/**
	 *	@brief conversion from RGB8 to RGBA8
	 */
	struct T_RGB8_Color {
		typedef uint8_t TDataType[3]; /**< @brief raw color data type (as stored in file) */

		/**
		 *	@brief performs conversion from RGB8 color to RGBA8
		 *	@param[in] r_t_color is the color to be converted
		 *	@return Returns RGBA8 color representation of the supplied color.
		 */
		inline uint32_t operator()(const TDataType &r_t_color) const
		{
			return ((uint32_t)r_t_color[0] << 16) | ((uint32_t)r_t_color[1] << 8) |
				(uint32_t)r_t_color[2] | 0xff000000;
		}
	};

	/**
	 *	@brief conversion from BGRA8 to RGBA8
	 */
	struct T_BGRA8_Color {
		typedef uint32_t TDataType; /**< @brief raw color data type (as stored in file) */

		/**
		 *	@brief performs conversion from BGRA8 color to RGBA8
		 *	@param[in] r_t_color is the color to be converted
		 *	@return Returns RGBA8 color representation of the supplied color.
		 */
		inline uint32_t operator()(const TDataType &r_t_color) const
		{
			return (r_t_color & 0xff00ff00) | ((r_t_color & 0xff0000) >> 16) |
				((r_t_color & 0xff) << 16);
		}
	};

	/**
	 *	@brief conversion from RGB5 to RGBA8
	 */
	struct T_RGB5_Color {
		typedef uint16_t TDataType; /**< @brief raw color data type (as stored in file) */

		/**
		 *	@brief performs conversion from RGB5 color to RGBA8
		 *	@param[in] r_t_color is the color to be converted
		 *	@return Returns RGBA8 color representation of the supplied color.
		 */
		inline uint32_t operator()(const TDataType &r_t_color) const
		{
			return n_5_to_8((r_t_color >> 10) & 0x1f) |
				  (n_5_to_8((r_t_color >> 5) & 0x1f) << 8) |
				  (n_5_to_8(r_t_color & 0x1f) << 16) | 0xff000000;
		}

	protected:
		/**
		 *	@brief converts value from 5-bit range to 8-bit range
		 *	@param[in] n_x is value in 5-bit range (0 - 31)
		 *	@return Returns corresponding value in 8-bit range (0 - 255).
		 */
		static inline int n_5_to_8(int n_x)
		{
			return (n_x * 255 / 31) & 0xff;
		}
	};

	/**
	 *	@brief information about a TGA image
	 */
	struct TTGAInfo {
		bool b_upside_down; /**< @brief image is stored upside down */
		bool b_compressed; /**< @brief RLE compression flag */
		bool b_grayscale; /**< @brief grayscale flag */
		bool b_alpha; /**< @brief alpha channel flag */
		int n_palette_entry_num; /**< @brief number of palette entries or 0 for true color */
		int n_bit_depth; /**< @brief number of bits per pixel (or per palette color) */
		int n_x; /**< @brief "left" from TGA header */
		int n_y; /**< @brief "top" from TGA header */
		int n_width; /**< @brief image width */
		int n_height; /**< @brief image height */
		int n_id_length; /**< @brief length of image id */
		char p_id[64]; /**< @brief image id, cropped to the maximum of 63 characters, always null-terminated */
	};

public:
	/**
	 *	@brief reads image header from a file and returns image properties
	 *
	 *	@param[out] r_t_info is structure to be filled with image information
	 *	@param[in] p_s_filename is input file name
	 *
	 *	@return Returns true on success, false on failure.
	 */
	static bool Get_ImageInfo(TTGAInfo &r_t_info, const char *p_s_filename);

	/**
	 *	@brief loads an image from a file
	 *	@param[in] p_s_filename is input file name
	 *	@return Returns pointer to bitmap object or 0 if loading failed.
	 */
	static TBmp *p_Load_TGA(const char *p_s_filename);

	/**
	 *	@brief saves an image to a file
	 *
	 *	@param[in] p_s_filename is output file name
	 *	@param[in] r_t_bmp is the image to be saved (if r_t_bmp.b_alpha is set,
	 *		the image is written as RGBA, otherwise as RGB. also, if r_t_bmp.b_grayscale
	 *		is set (while r_t_bmp.b_alpha is not), the image is written
	 *		as RLE - compressed grayscale).
	 *	@param[in] b_BGRA is set if the image is BGRA (images loaded by p_Load_TGA() are BGRA)
	 *	@param[in] b_RGB_RLE is RLE compression flag for RGB images (grayscale are always RLE)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note The t_bmp.n_former_bpp field is ignored, images are always saved as 8bpp.
	 */
	static bool Save_TGA(const char *p_s_filename, const TBmp &r_t_bmp,
		bool b_BGRA, bool b_RGB_RLE = false);

protected:
	/**
	 *	@brief extracts red color from rgba (used by Save_TGA() for grayscale conversion)
	 *	@param[in] n_rgba is RGBA color
	 *	@return Returns value of red channel.
	 */
	static inline uint8_t n_Red(uint32_t n_rgba);
};

#endif // !__TARGA_INCLUDED
