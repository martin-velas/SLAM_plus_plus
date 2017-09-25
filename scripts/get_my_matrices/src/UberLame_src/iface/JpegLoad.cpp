/*
								+---------------------------------+
								|                                 |
								|   ***  jpeglib interface  ***   |
								|                                 |
								|  Copyright  © -tHE SWINe- 2005  |
								|                                 |
								|          JpegLoad.cpp           |
								|                                 |
								+---------------------------------+
*/

/**
 *	@file iface/JpegLoad.cpp
 *	@date 2005
 *	@author -tHE SWINe-
 *	@brief jpeglib interface
 */

#include "../NewFix.h"
#include "../CallStack.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include "../MinMax.h"
#include "JpegLoad.h"
#include <stdexcept>

extern "C" {
#undef INT32
#define JPEG_INTERNAL_OPTIONS
#include <jpeglib.h>
}

#if RGB_RED == 0 && RGB_GREEN == 1 && RGB_BLUE == 2
#define __JPEGLIB_RGB
#elif RGB_RED == 2 && RGB_GREEN == 1 && RGB_BLUE == 0
#define __JPEGLIB_BGR
#else
#error "jpeglib mangled RGB color"
#endif

#if defined(_MSC_VER) && !defined(__MWERKS__)
#ifdef _DEBUG
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#pragma comment(lib, "jpegd_64.lib")
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#pragma comment(lib, "jpegd.lib")
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#else // _DEBUG
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#pragma comment(lib, "jpeg_64.lib")
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#pragma comment(lib, "jpeg.lib")
#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
#endif // _DEBUG
#endif // _MSC_VER && !__MWERKS__

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

static void my_error_exit(j_common_ptr cinfo)
{
	//(*cinfo->err->output_message)(cinfo);
	throw std::runtime_error("libjpeg operation failed");
}

TBmp *CJpegCodec::p_Load_JPEG(const char *p_s_filename)
{
	FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
	if(fopen_s(&p_fr, p_s_filename, "rb"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
	if(!(p_fr = fopen(p_s_filename, "rb")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		return 0;
	// open source file

	TBmp *p_bitmap = 0;
	struct jpeg_error_mgr jerr;
	struct jpeg_decompress_struct cinfo;
	try {
		cinfo.err = jpeg_std_error(&jerr);
		jerr.error_exit = &my_error_exit;
		// Step 1: allocate and initialize JPEG decompression object

		jpeg_create_decompress(&cinfo); // Now we can initialize the JPEG decompression object.
		jpeg_stdio_src(&cinfo, p_fr); // Step 2: specify data source (eg, a file)
		jpeg_read_header(&cinfo, TRUE); // Step 3: read file parameters with jpeg_read_header()

		//cinfo.dct_method = JDCT_IFAST;
		cinfo.dct_method = JDCT_FLOAT; // prefer precision
		cinfo.do_fancy_upsampling = FALSE;
		cinfo.do_block_smoothing = FALSE;
		if(cinfo.jpeg_color_space == JCS_YCCK)
			cinfo.out_color_space = JCS_CMYK;
		else
			cinfo.out_color_space = JCS_RGB;
		// Step 4: set parameters for decompression

		jpeg_start_decompress(&cinfo); // Step 5: Start decompressor

		int row_stride = cinfo.output_width * cinfo.output_components; // JSAMPLEs per row in output buffer
		JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
		_ASSERTE(buffer);
		// Make a one-row-high sample array that will go away when done with image

		if(!(p_bitmap = TBmp::p_Alloc(cinfo.output_width, cinfo.output_height,
		   cinfo.jpeg_color_space == JCS_GRAYSCALE, false, cinfo.data_precision))) {
			throw std::runtime_error("failed to allocate bitmap"); // throw to prevent the compiler from optimizing the catch block away
			/*fclose(p_fr);
			return 0;*/
		}
		// alloc mem for bitmap

		if(cinfo.jpeg_color_space == JCS_YCCK) {
			while(cinfo.output_scanline < cinfo.output_height) {
				jpeg_read_scanlines(&cinfo, buffer, 1);
				JSAMPROW ptr = buffer[0];
				for(int i = 0, w = p_bitmap->n_width; i < w; i ++) {
					uint8_t _b = *ptr ++;
					uint8_t _g = *ptr ++;
					uint8_t _r = *ptr ++;
					uint8_t k = *ptr ++;

					int r = max(0, min(255, k - ((255 - _r) * k) / 256));
					int g = max(0, min(255, k - ((255 - _g) * k) / 256));
					int b = max(0, min(255, k - ((255 - _b) * k) / 256));

					p_bitmap->p_buffer[i + w * (cinfo.output_scanline - 1)] = 
#ifdef __JPEGLIB_BGR
						0xff000000U | (b << 16) | (g << 8) | r;
#else // __JPEGLIB_BGR
						0xff000000U | (r << 16) | (g << 8) | b;
#endif // __JPEGLIB_BGR
				}
			}
		} else {
			while(cinfo.output_scanline < cinfo.output_height) {
				jpeg_read_scanlines(&cinfo, buffer, 1);
				JSAMPROW ptr = buffer[0];
				for(int i = 0, w = p_bitmap->n_width; i < w; i ++) {
					int b = *ptr ++;
					int g = *ptr ++;
					int r = *ptr ++;
					p_bitmap->p_buffer[i + w * (cinfo.output_scanline - 1)] = 
#ifdef __JPEGLIB_BGR
						0xff000000U | (b << 16) | (g << 8) | r;
#else // __JPEGLIB_BGR
						0xff000000U | (r << 16) | (g << 8) | b;
#endif // __JPEGLIB_BGR
				}
			}
		}
		// Step 6: while (scan lines remain to be read)
		//           jpeg_read_scanlines(...);

		jpeg_finish_decompress(&cinfo); // Step 7: Finish decompression
		jpeg_destroy_decompress(&cinfo); // Step 8: Release JPEG decompression object
		fclose(p_fr);
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		jpeg_destroy_decompress(&cinfo);
		if(p_bitmap)
			p_bitmap->Delete();
		fclose(p_fr);
		return 0;
	}

	return p_bitmap;
}

bool CJpegCodec::Save_JPEG(const char *p_s_filename, const TBmp &r_bmp,
	bool b_BGRA, int n_colorspace, int n_quality, bool b_optimize, int n_subsampling_x,
	int n_subsampling_y, bool b_allow_16bit_quant_tables,
	bool b_progressive, int n_restart_interval)
{
#ifdef __JPEGLIB_BGR
	b_BGRA = !b_BGRA;
	// if jpeglib is configured for BGR instead of RGB, flip the flag
#endif // __JPEGLIB_BGR

	const TBmp *p_bmp = &r_bmp; // ...
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	jerr.error_exit = &my_error_exit;
	cinfo.err = jpeg_std_error(&jerr);
	FILE *p_fw = 0;
	try {
		jpeg_create_compress(&cinfo);
		// Initialize the JPEG compression object with custom error handling.

		cinfo.in_color_space = JCS_RGB; // jpeg_set_defaults() wouldn't work otherwise
		jpeg_set_defaults(&cinfo);
		// Initialize JPEG parameters.

#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fw, p_s_filename, "wb")) {
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fw = fopen(p_s_filename, "wb"))) {
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			//return false;
			throw std::runtime_error("failed to open output file"); // throw to prevent the compiler from optimizing the catch block away
		}
		// Open the output file.

		{
			if(n_colorspace == jcolor_YCCK) {
				cinfo.in_color_space = JCS_YCCK; // jpeglib can't convert RGB to YCCK, have to convert it by hand
				cinfo.input_components = 4;
			} else {
				cinfo.in_color_space = JCS_RGB;
				cinfo.input_components = 3;
			}
			cinfo.data_precision = 8;
			cinfo.image_width = p_bmp->n_width;
			cinfo.image_height = p_bmp->n_height;
			// set input image info
		}
		// Read the input file header to obtain file size & colorspace.

		jpeg_default_colorspace(&cinfo);
		// Now that we know input colorspace, fix colorspace-dependent defaults

		{
			cinfo.err->trace_level = -1;
			// quiet

			for(int i = 0; i < NUM_QUANT_TBLS; ++ i)
				cinfo.q_scale_factor[i] = jpeg_quality_scaling(n_quality);
			jpeg_default_qtables(&cinfo, !b_allow_16bit_quant_tables);
			// set quality, generate quanting tables based on it

			if(n_subsampling_x < 1 || n_subsampling_x > 4)
				return false;
			if(n_subsampling_y < 1 || n_subsampling_y > 4)
				return false;
			cinfo.comp_info[0].h_samp_factor = n_subsampling_x;
			cinfo.comp_info[0].v_samp_factor = n_subsampling_y;
			for(int i = 1; i < MAX_COMPONENTS; ++ i) {
				cinfo.comp_info[i].h_samp_factor = 1;
				cinfo.comp_info[i].v_samp_factor = 1;
			}
			// set subsampling

			switch(n_colorspace) {
			case jcolor_Gray:
				jpeg_set_colorspace(&cinfo, JCS_GRAYSCALE);
				break;
			case jcolor_YCbCr:
				jpeg_set_colorspace(&cinfo, JCS_YCbCr);
				break;
			case jcolor_YCCK:
				jpeg_set_colorspace(&cinfo, JCS_YCCK);
				break;
			default:
				return false;
			}
			// set output colorspace

			cinfo.optimize_coding = (b_optimize)? TRUE : FALSE;
			// optimize huffman codes

			if(n_restart_interval < 0 || n_restart_interval > 65535)
				return false;
			cinfo.restart_interval = (unsigned int)n_restart_interval;
			cinfo.restart_in_rows = 0;
			// set restart interval

			if(b_progressive)
				jpeg_simple_progression(&cinfo);
			// set progressive
		}
		// Adjust default compression parameters by re-parsing the options

		jpeg_stdio_dest(&cinfo, p_fw);
		// Specify data destination for compression

		JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE,
			cinfo.image_width * cinfo.input_components, 1);
		_ASSERTE(buffer);
		// Make a one-row-high sample array that will go away when done with image

		jpeg_start_compress(&cinfo, TRUE);
		// Start compressor

		const int n_red_shift = (b_BGRA)? 0 : 16;
		const int n_blue_shift = (b_BGRA)? 16 : 0;
		// handle BGRA

		const uint32_t *p_current_pixel = p_bmp->p_buffer;
		if(n_colorspace == jcolor_YCCK) {
			while(cinfo.next_scanline < cinfo.image_height) {
				JSAMPROW ptr = buffer[0];
				for(int i = 0, w = p_bmp->n_width; i < w; ++ i) {
					uint32_t n_pixel = *p_current_pixel ++;
					int r = (n_pixel >> n_red_shift) & 0xff;
					int g = (n_pixel >> 8) & 0xff;
					int b = (n_pixel >> n_blue_shift) & 0xff;

					/*
					r = k - ck		c = (k - r) / k
					g = k - mk		m = (k - g) / k
					b = k - yk		y = (k - b) / k

					(k - r) / k = c

					criterion 1: cmy shouldn't be negative:
						k >= max(r,g,b)
					criterion 2: cmy shouldn't be larger than 255:
						(k - max(r,g,b)) * 255 / k <= 255
						-max(r,g,b) * 255 / k + k * 255 / k <= 255
						-max(r,g,b) * 255 / k <= 0
						k > 0

					k = max(rgb) works, in case k is 0, cmy are 0 as well
					*/

					int k = max(r, max(g, b));
					int c = (k)? ((k - r) * 256) / k : 0;
					int m = (k)? ((k - g) * 256) / k : 0;
					int y = (k)? ((k - b) * 256) / k : 0;

					int Y = max(0, min(255, int(.299f * c + .587f * m + .114f * y)));
					int Cb = max(0, min(255, int(-.1687f * c + -.3313f * m + .5f * y + 128)));
					int Cr = max(0, min(255, int(.5f * c + -.4187f * m + -.0813f * y + 128)));

					*ptr ++ = (JSAMPLE)Y;
					*ptr ++ = (JSAMPLE)Cb;
					*ptr ++ = (JSAMPLE)Cr;
					*ptr ++ = (JSAMPLE)k;
				}
				jpeg_write_scanlines(&cinfo, buffer, 1);
			}
		} else {
			while(cinfo.next_scanline < cinfo.image_height) {
				JSAMPROW ptr = buffer[0];
				for(int i = 0, w = p_bmp->n_width; i < w; ++ i) {
					uint32_t n_pixel = *p_current_pixel ++;
					*ptr ++ = (JSAMPLE)((n_pixel >> n_red_shift) & 0xff);
					*ptr ++ = (JSAMPLE)((n_pixel >> 8) & 0xff);
					*ptr ++ = (JSAMPLE)((n_pixel >> n_blue_shift) & 0xff);
				}
				jpeg_write_scanlines(&cinfo, buffer, 1);
			}
		}
		// Process data

		jpeg_finish_compress(&cinfo);
		jpeg_destroy_compress(&cinfo);
		// Finish compression and release memory

		if(p_fw)
			fclose(p_fw);
		// Close files, if we opened them
#ifdef _DEBUG
	} catch(std::runtime_error &r_exc) {
		fprintf(stderr, "error: %s\n", r_exc.what());
#else // _DEBUG
	} catch(std::runtime_error&) {
#endif // _DEBUG
		jpeg_destroy_compress(&cinfo);
		if(p_fw)
			fclose(p_fw);
		return 0;
	}

	return true;
}
