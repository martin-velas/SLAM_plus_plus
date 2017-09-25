/*
								+--------------------------------+
								|                                |
								|   ***  libpng interface  ***   |
								|                                |
								|  Copyright © -tHE SWINe- 2005  |
								|                                |
								|          PNGLoad.cpp           |
								|                                |
								+--------------------------------+
*/

/**
 *	@file iface/PNGLoad.cpp
 *	@date 2005
 *	@author -tHE SWINe-
 *	@brief libpng interface
 */

#include "../NewFix.h"
#include "../CallStack.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "PNGLoad.h"
#include <stdexcept>

#if defined(_MSC_VER) && !defined(__MWERKS__)
#ifdef _DEBUG
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#pragma comment(lib, "zlibd64.lib")
#pragma comment(lib, "libpngd64.lib")
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#pragma comment(lib, "zlibd.lib")
#pragma comment(lib, "libpngd.lib")
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _DEBUG
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#pragma comment(lib, "zlib64.lib")
#pragma comment(lib, "libpng64.lib")
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#pragma comment(lib, "zlib.lib")
#pragma comment(lib, "libpng.lib")
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#endif // _DEBUG
#endif // _MSC_VER && !__MWERKS__

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

extern "C" {
#define PNG_EXPORT(a,b) a b
#include <png.h>
}

/*
 *								=== CPngCodec::CInternal ===
 */

class CPngCodec::CInternal {
public:
	struct TDataBuffer {
		const uint8_t *p_ptr, *p_end;

		inline TDataBuffer(const void *_p_ptr, size_t n_size)
			:p_ptr((const uint8_t*)_p_ptr), p_end(((const uint8_t*)_p_ptr) + n_size)
		{}
	};

public:
	static TBmp *p_Load_PNG_int(png_structp p_png_ptr);
	static bool GetInfo_int(TImageInfo &r_t_info, png_structp p_png_ptr);
	
	static bool Save_PNG_int(png_structp png_ptr,
		const TBmp &r_t_bitmap, bool b_BGRA, bool b_interlace);

	static void ReadData_Callback(png_structp p_png_ptr, png_bytep p_out_buffer,
	   png_size_t n_byte_count_to_read);
	static void WriteData_Callback(png_structp p_png_ptr,
		png_bytep p_data, png_size_t n_size); // throw(std::bad_alloc)
	static void Flush_Callback(png_structp p_png_ptr); // do nothing
	static void png_my_error(png_structp p_png_ptr, png_const_charp p_s_message);
	static void png_my_warning(png_structp p_png_ptr, png_const_charp p_s_message);
};

void CPngCodec::CInternal::ReadData_Callback(png_structp p_png_ptr,
	png_bytep p_out_buffer, png_size_t n_byte_count_to_read)
{
	if(p_png_ptr->io_ptr == NULL) {
		throw std::runtime_error("png reader error: p_png_ptr->io_ptr == NULL");
		return;
	}
	// make sure the reader object was specified

	TDataBuffer &r_buffer = *(TDataBuffer*)(p_png_ptr->io_ptr);
	if(r_buffer.p_ptr + n_byte_count_to_read > r_buffer.p_end) {
		throw std::runtime_error("png reader error: insufficient data");
		return;
	}
	// make sure there's enough data

	memcpy(p_out_buffer, r_buffer.p_ptr, n_byte_count_to_read);
	r_buffer.p_ptr += n_byte_count_to_read;
	// "read" the data
}

void CPngCodec::CInternal::WriteData_Callback(png_structp p_png_ptr,
	png_bytep p_data, png_size_t n_size) // throw(std::bad_alloc)
{
	if(p_png_ptr->io_ptr == NULL) {
		throw std::runtime_error("png writer error: p_png_ptr->io_ptr == NULL");
		return;
	}
	// make sure the reader object was specified

	std::vector<uint8_t> &r_dest_buffer = *(std::vector<uint8_t>*)(p_png_ptr->io_ptr);
	// get the destination buffer

	_ASSERTE(sizeof(uint8_t) == 1);
	const uint8_t *p_begin = (const uint8_t*)p_data;
	const uint8_t *p_end = p_begin + n_size;

	r_dest_buffer.insert(r_dest_buffer.end(), p_begin, p_end);
	// try to insert data into the dest buffer
}

void CPngCodec::CInternal::Flush_Callback(png_structp UNUSED(p_png_ptr)) // do nothing
{}

void CPngCodec::CInternal::png_my_error(png_structp p_png_ptr, png_const_charp p_s_message)
{
	throw std::runtime_error(p_s_message);
}

void CPngCodec::CInternal::png_my_warning(png_structp p_png_ptr, png_const_charp p_s_message)
{
#ifdef _DEBUG
	printf("libpng warning: %s - %s\n", p_s_message, (char*)png_get_error_ptr(p_png_ptr)); // debug
#endif // _DEBUG
}

static inline void this_throws_std_runtime_error() // throw(std::runtime_error)
{
	uint8_t *p_ptr = new(std::nothrow) uint8_t[1];
	if(!p_ptr)
		throw std::runtime_error("failed to alloc 1 byte");
	delete[] p_ptr;
	// msvc optimizer workaround - otherwise it can't see that std::runtime_error
	// could be caught here and optimizes the catch block away
}

bool CPngCodec::CInternal::GetInfo_int(TImageInfo &r_t_info, png_structp p_png_ptr)
{
	png_infop p_png_info;
	if(!(p_png_info = png_create_info_struct(p_png_ptr))) {
		png_destroy_read_struct(&p_png_ptr, NULL, NULL);
		return 0;
	}

	png_infop p_end_info;
	if(!(p_end_info = png_create_info_struct(p_png_ptr))) {
		png_destroy_read_struct(&p_png_ptr, &p_png_info, NULL);
		return 0;
	}

	unsigned long n_width, n_height;
	int n_palette_entry_num, n_orig_bit_depth, n_color_type;
	try {
		this_throws_std_runtime_error();
		// msvc optimizer workaround - otherwise it can't see that std::runtime_error
		// could be caught here and optimizes the catch block away

		png_read_info(p_png_ptr, p_png_info);

		int n_interlace_type;
		png_get_IHDR(p_png_ptr, p_png_info, &n_width, &n_height,
			&n_orig_bit_depth, &n_color_type, &n_interlace_type, NULL, NULL);
		// get image header

		n_palette_entry_num = 0;
		if(n_color_type == PNG_COLOR_TYPE_PALETTE) {
			png_get_PLTE(p_png_ptr, p_png_info, 0, &n_palette_entry_num);
			n_orig_bit_depth = 8; // palettes are always stored as 24bit RGB (8 bit / pixel)
		}

		if(png_get_valid(p_png_ptr, p_png_info, PNG_INFO_tRNS)) {
			png_set_palette_to_rgb(p_png_ptr); // can't have palette & alpha, have to convert to RGB
			png_set_tRNS_to_alpha(p_png_ptr);
		}
		// transparency to alpha

		n_color_type = png_get_color_type(p_png_ptr, p_png_info);
		// update new values

		//png_read_end(p_png_ptr, NULL); // fires runtime_error, maybe it needs png_read_image() to be called
		// note this doesn't perform any cleanup, it just reads the info
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
		return 0;
	}
	// read the png from a file

	r_t_info.b_alpha = (n_color_type & PNG_COLOR_MASK_ALPHA) != 0;
	r_t_info.b_grayscale = (n_color_type == PNG_COLOR_TYPE_GRAY || n_color_type == PNG_COLOR_TYPE_GRAY_ALPHA);
	r_t_info.n_bit_depth = n_orig_bit_depth * (((r_t_info.b_grayscale)? 1 : 3) + ((r_t_info.b_alpha)? 1 : 0));
	r_t_info.n_width = n_width;
	r_t_info.n_height = n_height;
	r_t_info.n_palette_entry_num = n_palette_entry_num;
	// set bitmap properties

	png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
	// free scanlines

	return true;
}

TBmp *CPngCodec::CInternal::p_Load_PNG_int(png_structp p_png_ptr)
{
	png_infop p_png_info;
	if(!(p_png_info = png_create_info_struct(p_png_ptr))) {
		png_destroy_read_struct(&p_png_ptr, NULL, NULL);
		return 0;
	}

	png_infop p_end_info;
	if(!(p_end_info = png_create_info_struct(p_png_ptr))) {
		png_destroy_read_struct(&p_png_ptr, &p_png_info, NULL);
		return 0;
	}

	unsigned long n_width, n_height;
	int n_bit_depth, n_orig_bit_depth, n_color_type;
	png_bytep *p_scanline_list = 0;
	try {
		png_read_info(p_png_ptr, p_png_info);

		int n_interlace_type;
		png_get_IHDR(p_png_ptr, p_png_info, &n_width, &n_height,
			&n_bit_depth, &n_color_type, &n_interlace_type, NULL, NULL);
		n_orig_bit_depth = n_bit_depth;
		// get image header

		if(!(p_scanline_list = new(std::nothrow) png_bytep[n_height])) {
			throw std::runtime_error("failed to allocate p_scanline_list"); // use exception so that the compiler does not optimize the catch block away
			/*png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
			return 0;*/
		}
		// alloc scanline list

		for(unsigned long i = 0; i < n_height; i ++) {
			p_scanline_list[i] = (png_bytep)png_malloc(p_png_ptr,
				png_get_rowbytes(p_png_ptr, p_png_info));	
		}
		png_read_image(p_png_ptr, p_scanline_list);
		// decompress image

		if(n_color_type == PNG_COLOR_TYPE_PALETTE)
			png_set_palette_to_rgb(p_png_ptr);
		// don't want palette

		if(n_color_type == PNG_COLOR_TYPE_GRAY && n_bit_depth < 8)
			png_set_gray_1_2_4_to_8(p_png_ptr);
		// gray -> 8 bits

		if(png_get_valid(p_png_ptr, p_png_info, PNG_INFO_tRNS))
			png_set_tRNS_to_alpha(p_png_ptr);
		// transparency to alpha

		if(n_bit_depth == 16)
			png_set_strip_16(p_png_ptr);
		// 16 bit -> 8 bit

		n_bit_depth = png_get_bit_depth(p_png_ptr, p_png_info);
		n_color_type = png_get_color_type(p_png_ptr, p_png_info);
		// update new values

		switch(n_color_type) {
		case PNG_COLOR_TYPE_PALETTE: // (bit depths 1, 2, 4, 8)
			break;
		case PNG_COLOR_TYPE_GRAY:
		case PNG_COLOR_TYPE_RGB_ALPHA:
		case PNG_COLOR_TYPE_GRAY_ALPHA:
		case PNG_COLOR_TYPE_RGB:
			if(n_bit_depth == 8) // make sure we have 8bits per pixel
				break;
			// this case intentionally falls trough
		default:
			for(unsigned long i = 0; i < n_height; i ++)
				png_free(p_png_ptr, p_scanline_list[i]);	
			delete[] p_scanline_list;
			png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
			return 0;
		}
		// see what kind of color do we have

		png_read_end(p_png_ptr, p_png_info);
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		if(p_scanline_list) { // this was a bug, if the file is damaged, can throw before this is allocated, n_height might be uninitialized
			for(unsigned long i = 0; i < n_height; i ++)
				png_free(p_png_ptr, p_scanline_list[i]);	
			delete[] p_scanline_list;
		}
		png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info); // png ptr always exists priot to calling this function
		return 0;
	}
	// read the png from a file

	TBmp *p_bmp;
	if(!(p_bmp = TBmp::p_Alloc(n_width, n_height, n_color_type == PNG_COLOR_TYPE_GRAY ||
	   n_color_type == PNG_COLOR_TYPE_GRAY_ALPHA, (n_color_type & PNG_COLOR_MASK_ALPHA) != 0,
	   n_orig_bit_depth))) {
		for(unsigned long i = 0; i < n_height; i ++)
			png_free(p_png_ptr, p_scanline_list[i]);	
		delete[] p_scanline_list;
		png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
		return 0;
	}
	// alloc bitmap

	uint32_t *p_buffer = p_bmp->p_buffer;
	switch(n_color_type) {
	case PNG_COLOR_TYPE_GRAY:
		{
			for(unsigned long y = 0; y < n_height; ++ y) {
				for(unsigned long x = 0; x < n_width; ++ x, ++ p_buffer) {
					uint32_t n_gray = (uint32_t)(p_scanline_list[y])[x];
					*p_buffer = 0xff000000 | n_gray | (n_gray << 8) | (n_gray << 16);
				}
			}
		}
		break;
	case PNG_COLOR_TYPE_RGB_ALPHA:
		{
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += 4)
					*p_buffer = p_scanline[0] | (p_scanline[1] << 8) | (p_scanline[2] << 16) | (p_scanline[3] << 24);
			}
		}
		break;
	case PNG_COLOR_TYPE_GRAY_ALPHA:
		{
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += 2) {
					*p_buffer = p_scanline[0] | (p_scanline[0] << 8) | (p_scanline[0] << 16) | (p_scanline[1] << 24);
				}
			}
		}
		break;
	case PNG_COLOR_TYPE_RGB:
		{
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += 3) {
					*p_buffer = 0xff000000 | p_scanline[0] | (p_scanline[1] << 8) | (p_scanline[2] << 16);
				}
			}
		}
		break;
	case PNG_COLOR_TYPE_PALETTE:
		if(n_bit_depth == 8) {
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline ++) {
					*p_buffer = 0xff000000 | (p_png_ptr->palette[*p_scanline].blue << 16) |
						(p_png_ptr->palette[*p_scanline].green << 8) |
						(p_png_ptr->palette[*p_scanline].red);
				}
			}
		} else if(n_bit_depth == 4) {
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += (x & 1)) {
					uint8_t n_pal_entry = (*p_scanline >> (4 * (x & 1))) & 0xf;
					*p_buffer = 0xff000000 | (p_png_ptr->palette[n_pal_entry].blue << 16) |
						(p_png_ptr->palette[n_pal_entry].green << 8) |
						(p_png_ptr->palette[n_pal_entry].red);
				}
			}
		} else if(n_bit_depth == 2) {
			for(unsigned long y = 0; y < n_height; y ++) { // todo - find such an image! it's untested
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += !(x & 3)) {
					uint8_t n_pal_entry = (*p_scanline >> (2 * (x & 3))) & 0x3;
					*p_buffer = 0xff000000 | (p_png_ptr->palette[n_pal_entry].blue << 16) |
						(p_png_ptr->palette[n_pal_entry].green << 8) |
						(p_png_ptr->palette[n_pal_entry].red);
				}
			}
		} else if(n_bit_depth == 1) {
			for(unsigned long y = 0; y < n_height; y ++) {
				uint8_t *p_scanline = (uint8_t*)p_scanline_list[y];
				for(unsigned long x = 0; x < n_width; x ++, p_buffer ++, p_scanline += !(x & 7)) {
					uint8_t n_pal_entry = (*p_scanline >> (7 - (x & 7))) & 1;
					*p_buffer = 0xff000000 | (p_png_ptr->palette[n_pal_entry].blue << 16) |
						(p_png_ptr->palette[n_pal_entry].green << 8) |
						(p_png_ptr->palette[n_pal_entry].red);
				}
			}
		}
		break;
	}
	// copy the image (convert to ARGB8)

	for(unsigned long i = 0; i < n_height; i ++)
		png_free(p_png_ptr, p_scanline_list[i]);	
	delete[] p_scanline_list;
	png_destroy_read_struct(&p_png_ptr, &p_png_info, &p_end_info);
	// free scanlines

	return p_bmp;
}

bool CPngCodec::CInternal::Save_PNG_int(png_structp png_ptr,
	const TBmp &r_t_bitmap, bool b_BGRA, bool b_interlace)
{
	png_bytep *row_pointers = 0;
	png_infop info_ptr = 0;

	try {
		if(!(info_ptr = png_create_info_struct(png_ptr))) {
			png_destroy_write_struct(&png_ptr, png_infopp_NULL);
			return false;
		}
		// allocate/initialize the image information data

		if(r_t_bitmap.b_grayscale)
			b_interlace = false;
		// can't interlace grayscale (not implemented)

		int n_color_type = (r_t_bitmap.b_grayscale && !r_t_bitmap.b_alpha)? // hack - alpha forces RGB now
			((r_t_bitmap.b_alpha)? PNG_COLOR_TYPE_GRAY_ALPHA : PNG_COLOR_TYPE_GRAY) :
			((r_t_bitmap.b_alpha)? PNG_COLOR_TYPE_RGB_ALPHA : PNG_COLOR_TYPE_RGB);
		png_set_IHDR(png_ptr, info_ptr, r_t_bitmap.n_width, r_t_bitmap.n_height,
			8, n_color_type, (b_interlace)? PNG_INTERLACE_ADAM7 :
			PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
		// set image information (writes as RGBA8)

		png_color_8 sig_bit = {0};
		if(r_t_bitmap.b_grayscale && !r_t_bitmap.b_alpha) // hack - alpha forces RGB now
			sig_bit.gray = r_t_bitmap.n_former_bpc; // a grayscale image
		else {
			sig_bit.red = r_t_bitmap.n_former_bpc;
			sig_bit.green = r_t_bitmap.n_former_bpc;
			sig_bit.blue = r_t_bitmap.n_former_bpc;
			// if we are dealing with a color image
		}
		sig_bit.alpha = (r_t_bitmap.b_alpha)? r_t_bitmap.n_former_bpc : 0; // if the image has an alpha channel
		png_set_sBIT(png_ptr, info_ptr, &sig_bit);
		// write significant bit chunk
		
		// other optional chunks like cHRM, bKGD, tRNS, tIME, oFFs, pHYs,
		// note that if sRGB is present the gAMA and cHRM chunks must be ignored
		// on read and must be written in accordance with the sRGB profile

		png_write_info(png_ptr, info_ptr);
		// write the file header information

		//png_set_invert_mono(png_ptr); // invert monochrome pixels
		//png_set_shift(png_ptr, &sig_bit); // Shift the pixels up to a legal bit depth and fill in as appropriate to correctly scale the image
		//png_set_packing(png_ptr); // pack pixels into bytes
		if(!r_t_bitmap.b_grayscale && !b_BGRA)
			png_set_bgr(png_ptr); // flip BGR pixels to RGB
		if(!r_t_bitmap.b_alpha) {
			if(!r_t_bitmap.b_grayscale)
				png_set_filler(png_ptr, 0, PNG_FILLER_AFTER); // Get rid of filler (OR ALPHA) bytes
			// pack XRGB/RGBX/ARGB/RGBA into RGB (4 channels -> 3 channels). The second parameter is not used.
		} /*else
			png_set_swap_alpha(png_ptr);*/ // swap location of alpha bytes from ARGB to RGBA
		//png_set_swap(png_ptr); // swap bytes of 16-bit files to most significant byte first
		//png_set_packswap(png_ptr); // swap bits of 1, 2, 4 bit packed pixel formats
		// write transformations

		if(!r_t_bitmap.b_grayscale) {
			if(b_interlace)
				png_set_interlace_handling(png_ptr);
			// turn on interlace handling if you are not using png_write_image()

			if(!(row_pointers = new(std::nothrow) png_bytep[r_t_bitmap.n_height])) {
				throw std::runtime_error("failed to alloc row_pointers"); // prevent optimizer from optimizing away the catch block
				/*png_destroy_write_struct(&png_ptr, &info_ptr);
				fclose(fp);
				return false;*/
			}
			for(int y = 0; y < r_t_bitmap.n_height; ++ y)
				row_pointers[y] = (png_bytep)&r_t_bitmap.p_buffer[y * r_t_bitmap.n_width];
			// alloc and fill row pointers

			png_write_image(png_ptr, row_pointers);
			//  write the image
		} else {
			// interlacing is ignored here

			int n_pixel_size = (r_t_bitmap.b_alpha)? 4 : 1; // gray-alpha or gray
			png_byte *p_gray_buffer;
			if(!(p_gray_buffer = (new(std::nothrow) uint8_t[n_pixel_size * r_t_bitmap.n_width]))) {
				throw std::runtime_error("failed to alloc row_pointers"); // prevent optimizer from optimizing away the catch block
				/*png_destroy_write_struct(&png_ptr, &info_ptr);
				fclose(fp);
				return false;*/
			}
			row_pointers = (png_bytep*)p_gray_buffer; // so it gets automatically deleted
			// alloc one scanline

			if(r_t_bitmap.b_alpha) {
#if 0
				fprintf(stderr, "warning: grayscale alpha does not work in Save_PNG()\n");
				// grayscale encodes correctly (the image is not skewed), but alpha is not visible

				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x) {
						//p_row[x * 2 + 1] = uint8_t(p_scanline[x] /*& 0xff*/);
						p_row[x * 2 + 1] = uint8_t(p_scanline[x] >> 24);
						p_row[x * 2 + 0] = uint8_t(p_scanline[x] /*& 0xff*/);
					}
					png_bytep p_row_ptrs[2] = {p_gray_buffer, p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
#else // 0
				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x) {
						p_row[x * 4 + 3] = uint8_t(p_scanline[x] >> 24);
						p_row[x * 4 + 0] = uint8_t(p_scanline[x]);
						p_row[x * 4 + 1] = uint8_t(p_scanline[x]);
						p_row[x * 4 + 2] = uint8_t(p_scanline[x]);
					}
					png_bytep p_row_ptrs[1] = {p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
#endif // 0
			} else {
				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x)
						p_row[x] = uint8_t(p_scanline[x]);
					png_bytep p_row_ptrs[1] = {p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
				// pack to single-bytes
			}
			// convert scanlines to gray one by one, write out
		}
		// write the image

		png_write_end(png_ptr, info_ptr);
		// finish writing the rest of the file

		png_destroy_write_struct(&png_ptr, &info_ptr);
	} catch(std::bad_alloc&) {
#ifdef _DEBUG
		fprintf(stderr, "error: %s\n", "bad_alloc");
#endif // _DEBUG
		if(row_pointers)
			delete[] row_pointers;
		if(info_ptr)
			png_destroy_write_struct(&png_ptr, &info_ptr);
		return false;
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		if(row_pointers)
			delete[] row_pointers;
		if(info_ptr)
			png_destroy_write_struct(&png_ptr, &info_ptr);
		return false;
	}
	// write the image

	if(row_pointers)
		delete[] row_pointers;
	// cleanup

	return true;
}

/*
 *								=== ~CPngCodec::CInternal ===
 */

/*
 *								=== CPngCodec ===
 */

bool CPngCodec::Get_ImageInfo(TImageInfo &r_t_info, const char *p_s_filename)
{
	memset(&r_t_info, 0, sizeof(TImageInfo));
	// clear first

	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open input file

	png_structp p_png_ptr;
	if(!(p_png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	   &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning))) {
		fclose(p_fr);
		return 0;
	}
	// create png reader

	png_init_io(p_png_ptr, p_fr);
	// read from here

	bool b_result = CInternal::GetInfo_int(r_t_info, p_png_ptr);
	// get info

	fclose(p_fr);
	// close the file

	return b_result;
}

bool CPngCodec::Get_ImageInfo(TImageInfo &r_t_info, const void *p_data, size_t n_size)
{
	memset(&r_t_info, 0, sizeof(TImageInfo));
	// clear first

	png_structp p_png_ptr;
	if(!(p_png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	   &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning)))
		return 0;
	// create png reader

	CPngCodec::CInternal::TDataBuffer t_data(p_data, n_size);
	png_set_read_fn(p_png_ptr, &t_data, CPngCodec::CInternal::ReadData_Callback);
	// read from here

	return CInternal::GetInfo_int(r_t_info, p_png_ptr);
	// get info
}

TBmp *CPngCodec::p_Load_PNG(const char *p_s_filename)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return 0;
	// open input file

	png_structp p_png_ptr;
	if(!(p_png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	   &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning))) {
		fclose(p_fr);
		return 0;
	}
	// create png reader

	png_init_io(p_png_ptr, p_fr);
	// read from here

	TBmp *p_result = CInternal::p_Load_PNG_int(p_png_ptr);
	// load png

	fclose(p_fr);
	// close the file

	return p_result;
}

TBmp *CPngCodec::p_Load_PNG(const void *p_data, size_t n_size)
{
	png_structp p_png_ptr;
	if(!(p_png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
	   &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning)))
		return 0;
	// create png reader

	CPngCodec::CInternal::TDataBuffer t_data(p_data, n_size);
	png_set_read_fn(p_png_ptr, &t_data, CPngCodec::CInternal::ReadData_Callback);
	// read from here

	return CInternal::p_Load_PNG_int(p_png_ptr);
	// load png
}

bool CPngCodec::Save_PNG(std::vector<uint8_t> &r_dest_buffer, const TBmp &r_t_bitmap,
	bool b_BGRA /*= false*/, bool b_interlace /*= false*/)
{
	r_dest_buffer.clear(); // !!

	png_structp png_ptr;
	if(!(png_ptr = (png_create_write_struct(PNG_LIBPNG_VER_STRING,
	   NULL, &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning))))
		return false;
	// create and initialize the png_struct with the desired error handler

	png_set_write_fn(png_ptr, (void*)&r_dest_buffer, &CPngCodec::CInternal::WriteData_Callback,
		&CPngCodec::CInternal::Flush_Callback);
	// or do this

	return CInternal::Save_PNG_int(png_ptr, r_t_bitmap, b_BGRA, b_interlace);
	// use the common function
}

bool CPngCodec::Save_PNG(const char *p_s_filename, const TBmp &r_t_bitmap,
	bool b_BGRA /*= false*/, bool b_interlace /*= false*/)
{
	FILE *fp;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&fp, p_s_filename, "wb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(fp = fopen(p_s_filename, "wb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return false;
	// open the file

	png_structp png_ptr;
	if(!(png_ptr = (png_create_write_struct(PNG_LIBPNG_VER_STRING,
	   NULL, &CPngCodec::CInternal::png_my_error, &CPngCodec::CInternal::png_my_warning)))) {
		fclose(fp);
		return false;
	}
	// create and initialize the png_struct with the desired error handler

	png_init_io(png_ptr, fp);
	// set up the output control if you are using standard C streams

#if 1
	bool b_result = CInternal::Save_PNG_int(png_ptr, r_t_bitmap, b_BGRA, b_interlace);
#else // 1
	png_bytep *row_pointers = 0;
	png_infop info_ptr = 0;
	try {
		if(!(info_ptr = png_create_info_struct(png_ptr))) {
			fclose(fp);
			png_destroy_write_struct(&png_ptr, png_infopp_NULL);
			return false;
		}
		// allocate/initialize the image information data

		png_init_io(png_ptr, fp);
		// set up the output control if you are using standard C streams

		//png_set_write_fn(png_ptr, (void *)user_io_ptr, user_write_fn,
		//	user_IO_flush_function); // where user_io_ptr is a structure you want available to the callbacks
		// or do this

		if(r_t_bitmap.b_grayscale)
			b_interlace = false;
		// can't interlace grayscale (not implemented)

		int n_color_type = (r_t_bitmap.b_grayscale && !r_t_bitmap.b_alpha)? // hack - alpha forces RGB now
			((r_t_bitmap.b_alpha)? PNG_COLOR_TYPE_GRAY_ALPHA : PNG_COLOR_TYPE_GRAY) :
			((r_t_bitmap.b_alpha)? PNG_COLOR_TYPE_RGB_ALPHA : PNG_COLOR_TYPE_RGB);
		png_set_IHDR(png_ptr, info_ptr, r_t_bitmap.n_width, r_t_bitmap.n_height,
			8, n_color_type, (b_interlace)? PNG_INTERLACE_ADAM7 :
			PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
		// set image information (writes as RGBA8)

		png_color_8 sig_bit = {0};
		if(r_t_bitmap.b_grayscale && !r_t_bitmap.b_alpha) // hack - alpha forces RGB now
			sig_bit.gray = r_t_bitmap.n_former_bpc; // a grayscale image
		else {
			sig_bit.red = r_t_bitmap.n_former_bpc;
			sig_bit.green = r_t_bitmap.n_former_bpc;
			sig_bit.blue = r_t_bitmap.n_former_bpc;
			// if we are dealing with a color image
		}
		sig_bit.alpha = (r_t_bitmap.b_alpha)? r_t_bitmap.n_former_bpc : 0; // if the image has an alpha channel
		png_set_sBIT(png_ptr, info_ptr, &sig_bit);
		// write significant bit chunk
		
		// other optional chunks like cHRM, bKGD, tRNS, tIME, oFFs, pHYs,
		// note that if sRGB is present the gAMA and cHRM chunks must be ignored
		// on read and must be written in accordance with the sRGB profile

		png_write_info(png_ptr, info_ptr);
		// write the file header information

		//png_set_invert_mono(png_ptr); // invert monochrome pixels
		//png_set_shift(png_ptr, &sig_bit); // Shift the pixels up to a legal bit depth and fill in as appropriate to correctly scale the image
		//png_set_packing(png_ptr); // pack pixels into bytes
		if(!r_t_bitmap.b_grayscale && !b_BGRA)
			png_set_bgr(png_ptr); // flip BGR pixels to RGB
		if(!r_t_bitmap.b_alpha) {
			if(!r_t_bitmap.b_grayscale)
				png_set_filler(png_ptr, 0, PNG_FILLER_AFTER); // Get rid of filler (OR ALPHA) bytes
			// pack XRGB/RGBX/ARGB/RGBA into RGB (4 channels -> 3 channels). The second parameter is not used.
		} /*else
			png_set_swap_alpha(png_ptr);*/ // swap location of alpha bytes from ARGB to RGBA
		//png_set_swap(png_ptr); // swap bytes of 16-bit files to most significant byte first
		//png_set_packswap(png_ptr); // swap bits of 1, 2, 4 bit packed pixel formats
		// write transformations

		if(!r_t_bitmap.b_grayscale) {
			if(b_interlace)
				png_set_interlace_handling(png_ptr);
			// turn on interlace handling if you are not using png_write_image()

			if(!(row_pointers = new(std::nothrow) png_bytep[r_t_bitmap.n_height])) {
				throw std::runtime_error("failed to alloc row_pointers"); // prevent optimizer from optimizing away the catch block
				/*png_destroy_write_struct(&png_ptr, &info_ptr);
				fclose(fp);
				return false;*/
			}
			for(int y = 0; y < r_t_bitmap.n_height; ++ y)
				row_pointers[y] = (png_bytep)&r_t_bitmap.p_buffer[y * r_t_bitmap.n_width];
			// alloc and fill row pointers

			png_write_image(png_ptr, row_pointers);
			//  write the image
		} else {
			// interlacing is ignored here

			int n_pixel_size = (r_t_bitmap.b_alpha)? 4 : 1; // gray-alpha or gray
			png_byte *p_gray_buffer;
			if(!(p_gray_buffer = (new(std::nothrow) uint8_t[n_pixel_size * r_t_bitmap.n_width]))) {
				throw std::runtime_error("failed to alloc row_pointers"); // prevent optimizer from optimizing away the catch block
				/*png_destroy_write_struct(&png_ptr, &info_ptr);
				fclose(fp);
				return false;*/
			}
			row_pointers = (png_bytep*)p_gray_buffer; // so it gets automatically deleted
			// alloc one scanline

			if(r_t_bitmap.b_alpha) {
#if 0
				fprintf(stderr, "warning: grayscale alpha does not work in Save_PNG()\n");
				// grayscale encodes correctly (the image is not skewed), but alpha is not visible

				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x) {
						//p_row[x * 2 + 1] = uint8_t(p_scanline[x] /*& 0xff*/);
						p_row[x * 2 + 1] = uint8_t(p_scanline[x] >> 24);
						p_row[x * 2 + 0] = uint8_t(p_scanline[x] /*& 0xff*/);
					}
					png_bytep p_row_ptrs[2] = {p_gray_buffer, p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
#else // 0
				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x) {
						p_row[x * 4 + 3] = uint8_t(p_scanline[x] >> 24);
						p_row[x * 4 + 0] = uint8_t(p_scanline[x]);
						p_row[x * 4 + 1] = uint8_t(p_scanline[x]);
						p_row[x * 4 + 2] = uint8_t(p_scanline[x]);
					}
					png_bytep p_row_ptrs[1] = {p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
#endif // 0
			} else {
				for(int y = 0; y < r_t_bitmap.n_height; ++ y) {
					uint8_t *p_row = (uint8_t*)p_gray_buffer;
					const uint32_t *p_scanline = r_t_bitmap.p_buffer + y * r_t_bitmap.n_width;
					for(int x = 0; x < r_t_bitmap.n_width; ++ x)
						p_row[x] = uint8_t(p_scanline[x]);
					png_bytep p_row_ptrs[1] = {p_gray_buffer};
					png_write_rows(png_ptr, p_row_ptrs, 1);
				}
				// pack to single-bytes
			}
			// convert scanlines to gray one by one, write out
		}
		// write the image

		png_write_end(png_ptr, info_ptr);
		// finish writing the rest of the file

		png_destroy_write_struct(&png_ptr, &info_ptr);
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		fclose(fp);
		if(row_pointers)
			delete[] row_pointers;
		if(info_ptr)
			png_destroy_write_struct(&png_ptr, &info_ptr);
		return false;
	}
	// write the image

	if(row_pointers)
		delete[] row_pointers;
#endif // 1

	fclose(fp);
	// cleanup

	return b_result;
}

/*
 *								=== ~CPngCodec ===
 */
