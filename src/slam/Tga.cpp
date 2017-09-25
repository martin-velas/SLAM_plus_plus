/*
								+---------------------------------+
								|                                 |
								|        ***   Targa   ***        |
								|                                 |
								| Copyright  (c) -tHE SWINe- 2006 |
								|                                 |
								|             Tga.cpp             |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file src/slam/Tga.cpp
 *	@brief Targa bitmap image format support.
 *	@date 2006
 *	@author -tHE SWINe-
 *
 *	@date 2006-06-08
 *
 *	passed code revision
 *
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
 */

#include <vector>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h> // LONG_MAX
#include "slam/Debug.h"
#include "slam/Tga.h"

/*
 *								=== CTgaCodec ===
 */

/**
 *	@brief helper classes for TGA codec
 */
namespace targa {

/**
 *	@brief function for reading run-length compressed scanlines
 *
 *	@tparam TColorStruct is color conversion structure type
 *
 *	@param[in] t_color_converter is object, used for color conversion
 *	@param[out] p_output is output array for the scanline (allocated by the caller)
 *	@param[in] n_width is scanline width
 *	@param[in] p_fr is source file (opened for reading)
 *
 *	@return Returns true on success, false on failure.
 */
template <class TColorStruct>
static bool _ReadRLEScanline(TColorStruct t_color_converter,
	uint32_t *p_output, int n_width, FILE *p_fr)
{
	for(int i = 0; i < n_width;) {
		uint8_t n_count;
		if(fread(&n_count, sizeof(char), 1, p_fr) != 1)
			return false;
		if(n_count & 0x80) {
			if(i + (n_count = (n_count & 0x7f) + 1) > n_width)
				return false;
			typename TColorStruct::TDataType t_value;
			if(fread(&t_value, sizeof(typename TColorStruct::TDataType), 1, p_fr) != 1)
				return false;
			for(uint32_t *p_end = p_output + n_count, n_color =
			   t_color_converter(t_value); p_output < p_end;)
				*p_output ++ = n_color;
			// long block with constant color
		} else {
			if(i + ++ n_count > n_width)
				return false;
			typename TColorStruct::TDataType p_value[256], *p_cur_value = &p_value[0];
			if(fread(p_value, sizeof(typename TColorStruct::TDataType), n_count, p_fr) != n_count)
				return false;
			for(uint32_t *p_end = p_output + n_count; p_output < p_end;)
				*p_output ++ = t_color_converter(*p_cur_value ++);
			// block with varying pixels
		}
		i += n_count;
	}

	return true;
}


/**
 *	@brief function for reading scanlines
 *
 *	@tparam TColorStruct is color conversion structure type
 *
 *	@param[in] t_color_converter is object, used for color conversion
 *	@param[out] p_output is output array for the scanline (allocated by the caller)
 *	@param[in] n_width is scanline width
 *	@param[in] p_fr is source file (opened for reading)
 *
 *	@return Returns true on success, false on failure.
 */
template <class TColorStruct>
static bool _ReadScanline(TColorStruct t_color_converter,
	uint32_t *p_output, int n_width, FILE *p_fr)
{
	for(int i = 0, n_step = (n_width > 256)? 256 : n_width; i < n_width;
	   i += n_step, n_step = (n_width - i > 256)? 256 : n_width - i) {
		typename TColorStruct::TDataType p_value[256], *p_cur_value = &p_value[0];
		if(fread(p_value, sizeof(typename TColorStruct::TDataType),
		   n_step, p_fr) != (unsigned)n_step)
			return false;
		for(uint32_t *p_end = p_output + n_step; p_output < p_end;)
			*p_output ++ = t_color_converter(*p_cur_value ++);
		// block with varying pixels
	}

	return true;
}

#pragma pack(1)

/**
 *	@brief Targa file header
 */
struct TTgaHeader {
	uint8_t n_id_length; /**< @brief length of image id, in bytes */
	uint8_t n_palette_type; /**< @brief palette type */
	uint8_t n_image_type; /**< @brief image type (one of tga_*) */
	uint16_t n_first_color; /**< @brief base index of the colors in palette */
	uint16_t n_palette_colors; /**< @brief number of colors in palette */
	uint8_t n_palette_entry_size; /**< @brief size of palette entry (the color, not the index) */
	uint16_t n_left; /**< @brief display position of the image */
	uint16_t n_top; /**< @brief display position of the image  */
	uint16_t n_image_width; /**< @brief width of the image, in pixels */
	uint16_t n_image_height; /**< @brief height of the image, in pixels */
	uint8_t n_bpp; /**< @brief number of bits per pixel */
	uint8_t n_descriptor_bits; /**< @brief flags, describing the image (0x20 means the image is stored upside-down) */
};

#pragma pack()

/**
 *	@brief image type bit flags
 */
enum {
	tga_Compressed_Mask = 8, /**< @brief image is compressed */
	tga_ImageType_Mask = 7, /**< @brief mask of image type */
	tga_ColorMapped = 1, /**< @brief image is color-mapped (palette) */
	tga_RGB = 2, /**< @brief image is RGB(A) */
	tga_Grayscale = 3 /**< @brief image is grayscale */
};

/**
 *	@brief color conversion adapter for fast RLE compression
 */
class CRLEColorOp_Grey {
public:
	typedef uint8_t _TyData; /**< @brief type of data stored in a file */

public:
	/**
	 *	@brief masks-out relevant color bits for RLE compression
	 *	@param[in] n_rgba is input RGBA color
	 *	@return Returns color with only relevant bits for RLE compression.
	 */
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return uint8_t(n_rgba >> 16);
	}

	/**
	 *	@brief transforms RGBA color to the form to be written to the file
	 *
	 *	@param[out] r_n_pixel is data to be stored in a file
	 *	@param[in] n_rgba is input RGBA color
	 */
	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel = n_Mask(n_rgba);
	}
};

/*class CRLEColorOp_GreyAlpha {
public:
	typedef uint8_t _TyData[2];

public:
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return uint16_t(n_rgba >> 16); // alpha and red
	}

	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel[0] = uint8_t(n_rgba >> 16); // R
		r_n_pixel[1] = uint8_t(n_rgba >> 24); // A
	}
};*/ // does not exist in .tga

/**
 *	@brief color conversion adapter for fast RLE compression
 */
class CRLEColorOp_RGB {
public:
	typedef uint8_t _TyData[3]; /**< @brief type of data stored in a file */

public:
	/**
	 *	@brief masks-out relevant color bits for RLE compression
	 *	@param[in] n_rgba is input RGBA color
	 *	@return Returns color with only relevant bits for RLE compression.
	 */
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return n_rgba & 0xffffff;
	}

	/**
	 *	@brief transforms RGBA color to the form to be written to the file
	 *
	 *	@param[out] r_n_pixel is data to be stored in a file
	 *	@param[in] n_rgba is input RGBA color
	 */
	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel[0] = uint8_t(n_rgba);
		r_n_pixel[1] = uint8_t(n_rgba >> 8);
		r_n_pixel[2] = uint8_t(n_rgba >> 16);
	}
};

/**
 *	@brief color conversion adapter for fast RLE compression
 */
class CRLEColorOp_RGBA {
public:
	typedef uint8_t _TyData[4]; /**< @brief type of data stored in a file */

public:
	/**
	 *	@brief masks-out relevant color bits for RLE compression
	 *	@param[in] n_rgba is input RGBA color
	 *	@return Returns color with only relevant bits for RLE compression.
	 */
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return n_rgba;
	}

	/**
	 *	@brief transforms RGBA color to the form to be written to the file
	 *
	 *	@param[out] r_n_pixel is data to be stored in a file
	 *	@param[in] n_rgba is input RGBA color
	 */
	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel[0] = uint8_t(n_rgba);
		r_n_pixel[1] = uint8_t(n_rgba >> 8);
		r_n_pixel[2] = uint8_t(n_rgba >> 16);
		r_n_pixel[3] = uint8_t(n_rgba >> 24);
	}
};

/**
 *	@brief color conversion adapter for fast RLE compression
 */
class CRLEColorOp_BGR {
public:
	typedef uint8_t _TyData[3]; /**< @brief type of data stored in a file */

public:
	/**
	 *	@brief masks-out relevant color bits for RLE compression
	 *	@param[in] n_rgba is input RGBA color
	 *	@return Returns color with only relevant bits for RLE compression.
	 */
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return n_rgba & 0xffffff;
	}

	/**
	 *	@brief transforms RGBA color to the form to be written to the file
	 *
	 *	@param[out] r_n_pixel is data to be stored in a file
	 *	@param[in] n_rgba is input RGBA color
	 */
	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel[0] = uint8_t(n_rgba >> 16);
		r_n_pixel[1] = uint8_t(n_rgba >> 8);
		r_n_pixel[2] = uint8_t(n_rgba);
	}
};

/**
 *	@brief color conversion adapter for fast RLE compression
 */
class CRLEColorOp_BGRA {
public:
	typedef uint8_t _TyData[4]; /**< @brief type of data stored in a file */

public:
	/**
	 *	@brief masks-out relevant color bits for RLE compression
	 *	@param[in] n_rgba is input RGBA color
	 *	@return Returns color with only relevant bits for RLE compression.
	 */
	static inline uint32_t n_Mask(uint32_t n_rgba)
	{
		return n_rgba;
	}

	/**
	 *	@brief transforms RGBA color to the form to be written to the file
	 *
	 *	@param[out] r_n_pixel is data to be stored in a file
	 *	@param[in] n_rgba is input RGBA color
	 */
	static inline void Transform(_TyData &r_n_pixel, uint32_t n_rgba)
	{
		r_n_pixel[0] = uint8_t(n_rgba >> 16);
		r_n_pixel[1] = uint8_t(n_rgba >> 8);
		r_n_pixel[2] = uint8_t(n_rgba);
		r_n_pixel[3] = uint8_t(n_rgba >> 24);
	}
};

/**
 *	@brief encodes RLE-compressed scanline
 *
 *	@tparam CColorOp is color conversion adapter type
 *
 *	@param[in] p_fw is output file
 *	@param[in] n_remains is number of remaining pixels in the scanline to be encoded
 *	@param[in] p_ptr is pointer to the current pixel
 *
 *	@return Returns pointer to one past the last scanline pixel on success, 0 on failure.
 */
template <class CColorOp>
static const uint32_t *_p_Write_RLE_Scanline(FILE *p_fw,
	size_t n_remains, const uint32_t *p_ptr)
{
	if(n_remains > 128)
		n_remains = 128;
	bool b_compress = n_remains > 2 &&
		CColorOp::n_Mask(*p_ptr) == CColorOp::n_Mask(*(p_ptr + 1)) &&
		CColorOp::n_Mask(*p_ptr) == CColorOp::n_Mask(*(p_ptr + 2));
	size_t n_run_length = n_remains;
	for(size_t i = 1; i < n_remains; ++ i) {
		if(b_compress && CColorOp::n_Mask(*p_ptr) != CColorOp::n_Mask(*(p_ptr + i))) {
			n_run_length = i;
			break;
		} else if(!b_compress && i + 1 < n_remains &&
		   CColorOp::n_Mask(*(p_ptr + i - 1)) == CColorOp::n_Mask(*(p_ptr + i)) &&
		   CColorOp::n_Mask(*(p_ptr + i - 1)) == CColorOp::n_Mask(*(p_ptr + i + 1))) {
			n_run_length = i - 1;
			break;
		}
	}
	// determine run length and wheter to compress data

	uint8_t n_code = uint8_t(n_run_length - 1) | ((b_compress)? 0x80 : 0x00);
	if(fwrite(&n_code, sizeof(uint8_t), 1, p_fw) != 1)
		return 0;

	if(b_compress) {
		typename CColorOp::_TyData n_pixel;
		CColorOp::Transform(n_pixel, *p_ptr);
		p_ptr += n_run_length - 1; // "- 1" because of the ++ in the enclosing for
		if(fwrite(&n_pixel, sizeof(typename CColorOp::_TyData), 1, p_fw) != 1)
			return 0;
	} else {
		for(const uint32_t *p_end3 = p_ptr + n_run_length; p_ptr != p_end3; ++ p_ptr) {
			typename CColorOp::_TyData n_pixel;
			CColorOp::Transform(n_pixel, *p_ptr);
			if(fwrite(&n_pixel, sizeof(typename CColorOp::_TyData), 1, p_fw) != 1)
				return 0;
		}
		-- p_ptr; // ++ in the enclosing for
	}

	return p_ptr;
}

}
using namespace targa;

bool CTgaCodec::Get_ImageInfo(TTGAInfo &r_t_info, const char *p_s_filename)
{
	{
		TTGAInfo t_clear;// = {0}; // to avoid warnings
		memset(&t_clear, 0, sizeof(TTGAInfo));
		r_t_info = t_clear;
	}
	// clear the output

	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open file

	TTgaHeader t_header;
	if(fread(&t_header, sizeof(TTgaHeader), 1, p_fr) != 1) {
		fclose(p_fr);
		return false;
	}
	// read header

	r_t_info.n_x = t_header.n_left;
	r_t_info.n_y = t_header.n_top;
	r_t_info.n_width = t_header.n_image_width;
	r_t_info.n_height = t_header.n_image_height;
	// copy image dimensions

	size_t n_read_id_length = std::min(size_t(t_header.n_id_length),
		sizeof(r_t_info.p_id) - sizeof(r_t_info.p_id[0]));
	size_t n_skip_id_length = t_header.n_id_length - n_read_id_length;
	_ASSERTE(n_skip_id_length * sizeof(r_t_info.p_id[0]) <= LONG_MAX);
	if(fread(r_t_info.p_id, 1, n_read_id_length, p_fr) != n_read_id_length ||
	   fseek(p_fr, long(n_skip_id_length * sizeof(r_t_info.p_id[0])), SEEK_CUR) != 0) {
		fclose(p_fr);
		return false;
	}
	r_t_info.n_id_length = t_header.n_id_length / sizeof(r_t_info.p_id[0]);
	r_t_info.p_id[n_read_id_length / sizeof(r_t_info.p_id[0])] = 0; // null-terminate
	// read id

	fclose(p_fr);

	if((t_header.n_image_type & tga_ImageType_Mask) < tga_ColorMapped ||
	   (t_header.n_image_type & tga_ImageType_Mask) > tga_Grayscale)
		return false;
	// make sure image type is valid

	if((t_header.n_image_type & tga_ImageType_Mask) == tga_ColorMapped) {
		r_t_info.n_palette_entry_num = t_header.n_palette_colors;
		r_t_info.n_bit_depth = t_header.n_palette_entry_size;
	} else if((t_header.n_image_type & tga_ImageType_Mask) == tga_Grayscale) {
		t_header.n_palette_colors = 256; // fixme - is it already there?
		r_t_info.n_palette_entry_num = 256;
		r_t_info.n_bit_depth = 8;
	} else {
		r_t_info.n_palette_entry_num = 0;
		r_t_info.n_bit_depth = t_header.n_bpp;
	}
	// copy information about colors

	r_t_info.b_alpha = r_t_info.n_bit_depth > 24;
	r_t_info.b_compressed = (t_header.n_image_type & tga_Compressed_Mask) != 0;
	r_t_info.b_grayscale = (t_header.n_image_type & tga_ImageType_Mask) == tga_Grayscale;
	r_t_info.b_upside_down = (t_header.n_descriptor_bits & 0x20) != 0;
	// set flags

	return true;
}

TBmp *CTgaCodec::p_Load_TGA(const char *p_s_filename)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return 0;
	// open file

	TTgaHeader t_header;
	if(fread(&t_header, sizeof(TTgaHeader), 1, p_fr) != 1 ||
	   fseek(p_fr, t_header.n_id_length, SEEK_CUR) != 0) {
		fclose(p_fr);
		return 0;
	}
	// read header

	if((t_header.n_image_type & tga_ImageType_Mask) == tga_Grayscale)
		t_header.n_palette_colors = 256; // fixme - is it already there?

	uint32_t *p_palette = 0;
	if(t_header.n_palette_colors &&
	   !(p_palette = new(std::nothrow) uint32_t[t_header.n_palette_colors])) {
		fclose(p_fr);
		return 0;
	}
	// alloc memory for palette

	if(t_header.n_palette_colors &&
	   (t_header.n_image_type & tga_ImageType_Mask) == tga_ColorMapped) {
		bool b_result;
		if(t_header.n_palette_entry_size == 16) {
			b_result = _ReadScanline(T_RGB5_Color(), p_palette,
				t_header.n_palette_colors, p_fr);
		} else if(t_header.n_palette_entry_size == 24) {
			b_result = _ReadScanline(T_RGB8_Color(), p_palette,
				t_header.n_palette_colors, p_fr);
		} else if(t_header.n_palette_entry_size == 32) {
			b_result = _ReadScanline(T_BGRA8_Color(), p_palette,
				t_header.n_palette_colors, p_fr);
		} else
			b_result = false;
		if(!b_result) {
			delete[] p_palette;
			fclose(p_fr);
			return 0;
		}
	}
	// read palette

	if((t_header.n_image_type & tga_ImageType_Mask) == tga_Grayscale) {
		for(uint32_t *p_color = p_palette, *p_end = p_palette + 256, n_color = 0xff000000;
		   p_color < p_end; n_color += 0x00010101)
			*p_color ++ = n_color;
	}
	// grayscale pal

	TBmp *p_bitmap;
	if((t_header.n_image_type & tga_ImageType_Mask) < tga_ColorMapped ||
	   (t_header.n_image_type & tga_ImageType_Mask) > tga_Grayscale || !(p_bitmap = new(std::nothrow) TBmp)) {
		if(p_palette)
			delete[] p_palette;
		fclose(p_fr);
		return 0;
	}
	if(!(p_bitmap->p_buffer = new(std::nothrow) uint32_t[t_header.n_image_width *
	   t_header.n_image_height])) {
		if(p_palette)
			delete[] p_palette;
		delete p_bitmap;
		fclose(p_fr);
		return 0;
	}
	p_bitmap->n_width = t_header.n_image_width;
	p_bitmap->n_height = t_header.n_image_height;
	p_bitmap->b_grayscale = (t_header.n_image_type & tga_ImageType_Mask) == tga_Grayscale;
	p_bitmap->n_former_bpc = (p_bitmap->b_grayscale)? 8 :
		(((t_header.n_image_type & tga_ImageType_Mask) == tga_RGB)?
		t_header.n_bpp : t_header.n_palette_entry_size);
	p_bitmap->b_alpha = p_bitmap->n_former_bpc == 32;
	// alloc bitmap

	const int n_step = (t_header.n_descriptor_bits & 0x20)? t_header.n_image_width :
		-t_header.n_image_width;
	for(uint32_t *p_scanline = p_bitmap->p_buffer + ((t_header.n_descriptor_bits & 0x20)?
	   0 : t_header.n_image_width * (t_header.n_image_height - 1)), *p_end = p_bitmap->p_buffer +
	   ((t_header.n_descriptor_bits & 0x20)? t_header.n_image_width * t_header.n_image_height :
	   -t_header.n_image_width); p_scanline != p_end; p_scanline += n_step) {
		bool b_result;
		if((t_header.n_image_type & tga_ImageType_Mask) == tga_RGB) {
			if(t_header.n_image_type & tga_Compressed_Mask) {
				if(t_header.n_bpp == 16) {
					b_result = _ReadRLEScanline(T_RGB5_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else if(t_header.n_bpp == 24) {
					b_result = _ReadRLEScanline(T_RGB8_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else if(t_header.n_bpp == 32) {
					b_result = _ReadRLEScanline(T_BGRA8_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else
					b_result = false;
			} else {
				if(t_header.n_bpp == 16) {
					b_result = _ReadScanline(T_RGB5_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else if(t_header.n_bpp == 24) {
					b_result = _ReadScanline(T_RGB8_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else if(t_header.n_bpp == 32) {
					b_result = _ReadScanline(T_BGRA8_Color(), p_scanline,
						p_bitmap->n_width, p_fr);
				} else
					b_result = false;
			}
		} else {
			if(t_header.n_image_type & tga_Compressed_Mask) {
				b_result = _ReadRLEScanline(TPaletteColor<uint8_t>(p_palette,
					t_header.n_palette_colors, -t_header.n_first_color),
					p_scanline, p_bitmap->n_width, p_fr);
			} else {
				b_result = _ReadScanline(TPaletteColor<uint8_t>(p_palette,
					t_header.n_palette_colors, -t_header.n_first_color),
					p_scanline, p_bitmap->n_width, p_fr);
			}
		}
		if(!b_result) {
			if(p_palette)
				delete[] p_palette;
			fclose(p_fr);
			delete[] p_bitmap->p_buffer;
			delete p_bitmap;
			return 0;
		}
	}

	if(p_palette)
		delete[] p_palette;
	fclose(p_fr);
	// cleanup

	return p_bitmap;
}

inline uint8_t CTgaCodec::n_Red(uint32_t n_rgba)
{
	return uint8_t(n_rgba >> 16);
}

bool CTgaCodec::Save_TGA(const char *p_s_filename,
	const TBmp &r_t_bmp, bool b_BGRA, bool b_RGB_RLE /*= false*/)
{
	FILE *p_fw;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fw, p_s_filename, "wb"))
#else // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fw = fopen(p_s_filename, "wb")))
#endif // _MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;

	TTgaHeader t_header;
    memset(&t_header, 0, sizeof(TTgaHeader));
	if(r_t_bmp.b_grayscale && !r_t_bmp.b_alpha) {
		t_header.n_bpp = 8;
		t_header.n_image_type = tga_Grayscale | tga_Compressed_Mask; // grey
	} else {
		t_header.n_bpp = (r_t_bmp.b_alpha)? 32 : 24;
		t_header.n_image_type = tga_RGB; // rgb
		if(b_RGB_RLE)
			t_header.n_image_type |= tga_Compressed_Mask;
	}
	t_header.n_image_width = r_t_bmp.n_width;
	t_header.n_image_height = r_t_bmp.n_height;
	t_header.n_descriptor_bits = 0x20; // upside-down
	// create header

	if(fwrite(&t_header, sizeof(t_header), 1, p_fw) != 1) {
		fclose(p_fw);
		return false;
	}
	// write header

	const int n_width = r_t_bmp.n_width;
	const int n_height = r_t_bmp.n_height;
	// used frequently in the loops below

	if(t_header.n_image_type == tga_RGB) {
		if(!b_BGRA) {
			if(t_header.n_bpp == 24) {
				for(const uint32_t *p_ptr = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_ptr != p_end; ++ p_ptr) {
					uint32_t n_rgb = *p_ptr;
					uint8_t p_data[3] = {uint8_t(n_rgb), uint8_t(n_rgb >> 8), uint8_t(n_rgb >> 16)};
					if(fwrite(p_data, sizeof(uint8_t), 3, p_fw) != 3) {
						fclose(p_fw);
						return false;
					}
				}
				// RGB
			} else {
				_ASSERTE(t_header.n_bpp == 32);
				for(const uint32_t *p_ptr = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_ptr != p_end; ++ p_ptr) {
					uint32_t n_rgba = *p_ptr;
					uint8_t p_data[4] = {uint8_t(n_rgba), uint8_t(n_rgba >> 8),
						uint8_t(n_rgba >> 16), uint8_t(n_rgba >> 24)};
					if(fwrite(p_data, sizeof(uint8_t), 4, p_fw) != 4) {
						fclose(p_fw);
						return false;
					}
				}
				// RGBA
			}
		} else {
			if(t_header.n_bpp == 24) {
				for(const uint32_t *p_ptr = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_ptr != p_end; ++ p_ptr) {
					uint32_t n_rgb = *p_ptr;
					uint8_t p_data[3] = {uint8_t(n_rgb >> 16), uint8_t(n_rgb >> 8), uint8_t(n_rgb)};
					if(fwrite(p_data, sizeof(uint8_t), 3, p_fw) != 3) {
						fclose(p_fw);
						return false;
					}
				}
				// BGR
			} else {
				_ASSERTE(t_header.n_bpp == 32);
				for(const uint32_t *p_ptr = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_ptr != p_end; ++ p_ptr) {
					uint32_t n_rgba = *p_ptr;
					uint8_t p_data[4] = {uint8_t(n_rgba >> 16), uint8_t(n_rgba >> 8),
						uint8_t(n_rgba), uint8_t(n_rgba >> 24)};
					if(fwrite(p_data, sizeof(uint8_t), 4, p_fw) != 4) {
						fclose(p_fw);
						return false;
					}
				}
				// BGRA
			}
		}
	} else if(t_header.n_image_type == (tga_RGB | tga_Compressed_Mask)) {
		if(!b_BGRA) {
			if(t_header.n_bpp == 24) {
				for(const uint32_t *p_scanline = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_scanline != p_end; p_scanline += n_width) {
					for(const uint32_t *p_ptr = p_scanline, *p_end2 = p_scanline + n_width;
					   p_ptr != p_end2; ++ p_ptr) {
						size_t n_remains = p_end2 - p_ptr;
						if(!(p_ptr = _p_Write_RLE_Scanline<CRLEColorOp_RGB>(p_fw, n_remains, p_ptr))) {
							fclose(p_fw); // !!
							return false;
						}
					}
				}
				// compressed RGB
			} else {
				_ASSERTE(t_header.n_bpp == 32);
				for(const uint32_t *p_scanline = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_scanline != p_end; p_scanline += n_width) {
					for(const uint32_t *p_ptr = p_scanline, *p_end2 = p_scanline + n_width;
					   p_ptr != p_end2; ++ p_ptr) {
						size_t n_remains = p_end2 - p_ptr;
						if(!(p_ptr = _p_Write_RLE_Scanline<CRLEColorOp_RGBA>(p_fw, n_remains, p_ptr))) {
							fclose(p_fw); // !!
							return false;
						}
					}
				}
				// compressed RGBA
			}
		} else {
			if(t_header.n_bpp == 24) {
				for(const uint32_t *p_scanline = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_scanline != p_end; p_scanline += n_width) {
					for(const uint32_t *p_ptr = p_scanline, *p_end2 = p_scanline + n_width;
					   p_ptr != p_end2; ++ p_ptr) {
						size_t n_remains = p_end2 - p_ptr;
						if(!(p_ptr = _p_Write_RLE_Scanline<CRLEColorOp_BGR>(p_fw, n_remains, p_ptr))) {
							fclose(p_fw); // !!
							return false;
						}
					}
				}
				// compressed BGR
			} else {
				_ASSERTE(t_header.n_bpp == 32);
				for(const uint32_t *p_scanline = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
				   (n_width * n_height); p_scanline != p_end; p_scanline += n_width) {
					for(const uint32_t *p_ptr = p_scanline, *p_end2 = p_scanline + n_width;
					   p_ptr != p_end2; ++ p_ptr) {
						size_t n_remains = p_end2 - p_ptr;
						if(!(p_ptr = _p_Write_RLE_Scanline<CRLEColorOp_BGRA>(p_fw, n_remains, p_ptr))) {
							fclose(p_fw); // !!
							return false;
						}
					}
				}
				// compressed BGRA
			}
		}
	} else {
		_ASSERTE(t_header.n_image_type == (tga_Grayscale | tga_Compressed_Mask));
		for(const uint32_t *p_scanline = r_t_bmp.p_buffer, *p_end = r_t_bmp.p_buffer +
		   (n_width * n_height); p_scanline != p_end; p_scanline += n_width) {
			for(const uint32_t *p_ptr = p_scanline, *p_end2 = p_scanline + n_width;
			   p_ptr != p_end2; ++ p_ptr) {
				size_t n_remains = p_end2 - p_ptr;
				if(!(p_ptr = _p_Write_RLE_Scanline<CRLEColorOp_Grey>(p_fw, n_remains, p_ptr))) {
					fclose(p_fw); // !!
					return false;
				}
				// everything in a simple cozy function

				/*if(n_remains > 128)
					n_remains = 128;
				bool b_compress = n_remains > 2 && n_Red(*p_ptr) == n_Red(*(p_ptr + 1)) &&
					n_Red(*p_ptr) == n_Red(*(p_ptr + 2));
				size_t n_run_length = n_remains;
				for(size_t i = 1; i < n_remains; ++ i) {
					if(b_compress && n_Red(*p_ptr) != n_Red(*(p_ptr + i))) {
						n_run_length = i;
						break;
					} else if(!b_compress && i + 1 < n_remains &&
					   n_Red(*(p_ptr + i - 1)) == n_Red(*(p_ptr + i)) &&
					   n_Red(*(p_ptr + i - 1)) == n_Red(*(p_ptr + i + 1))) {
						n_run_length = i - 1;
						break;
					}
				}
				// determine run length and wheter to compress data

				uint8_t n_code = uint8_t(n_run_length - 1) | ((b_compress)? 0x80 : 0x00);
				if(fwrite(&n_code, sizeof(uint8_t), 1, p_fw) != 1) {
					fclose(p_fw);
					return false;
				}

				if(b_compress) {
					uint8_t n_grey = n_Red(*p_ptr);
					p_ptr += n_run_length - 1;
					if(fwrite(&n_grey, sizeof(uint8_t), 1, p_fw) != 1) {
						fclose(p_fw);
						return false;
					}
				} else {
					for(const uint32_t *p_end3 = p_ptr + n_run_length; p_ptr != p_end3; ++ p_ptr) {
						uint8_t n_grey = n_Red(*p_ptr);
						if(fwrite(&n_grey, sizeof(uint8_t), 1, p_fw) != 1) {
							fclose(p_fw);
							return false;
						}
					}
					-- p_ptr; // ++ in the for command
				}*/
			}
		}
		// RLE grey
	}
	// write data

	return !fclose(p_fw);
}

/*
 *								=== ~CTgaCodec ===
 */
