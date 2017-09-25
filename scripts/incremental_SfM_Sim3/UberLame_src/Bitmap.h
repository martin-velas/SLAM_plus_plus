/*
								+--------------------------------+
								|                                |
								|       ***   Bitmap   ***       |
								|                                |
								|  Copyright © -tHE SWINe- 2011  |
								|                                |
								|            Bitmap.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __BITMAP_STRUCTURE_INCLUDED
#define __BITMAP_STRUCTURE_INCLUDED

/**
 *	@file Bitmap.h
 *	@date 2011
 *	@author -tHE SWINe-
 *	@brief a simple, easy to use bitmap class
 *
 *	@todo allow for use of shaders (textures) for line and triangle rasterization (even without z-buffer)
 *	t_odo write code for rendering axis-aligned rectangles
 *	@todo write code for rendering axis-aligned ellipses
 *	t_odo write code for rendering antialiased lines
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 *	@date 2012-11-02
 *
 *	Added subpixel precise and antialiased line rasterization routines.
 *	Added axis aligned rectangle rasterization routines.
 *
 *	@note All the new functionality is largerly untested - test it.
 *	t_odo Implement functions for flood fill, greyscale conversion.
 *	@todo Implement functions for image scaling, thresholding, basic morphology.
 *
 *	@date 2013-05-04
 *
 *	Fixed copy-paste error in thick line rasterization (missing a max() function).
 *
 *	@date 2015-06-15
 *
 *	Changed behaviot of TBmp::Make_Grayscale() which now preserves alpha (before, alpha was set to opaque).
 *	Added TBmp::Drop_Alpha().
 *
 *	Added TBmp::p_Begin() and TBmp::p_End() for more convenient implementation of point operators.
 *
 *	Fixed a bug in TBmp::p_Crop() which occured in case the crop rectangle was expanding the image.
 *
 */

/**
 *	@def __BMP_INCLUDED
 *	@brief legacy header guard name
 */
#define __BMP_INCLUDED

#include <new> // nothrow_t
#include <utility> // swap
#include <algorithm> // for_each
#include <string.h> // memcpy
#include <math.h>
#include <stdlib.h> // abs
#include "Integer.h"
#include "MinMax.h"

#if defined(_MSC_VER) && !defined(__MWERKS__) && !defined(for) && _MSC_VER <= 1200
#define for if(0) {} else for
#endif // _MSC_VER && !__MWERKS__ && !for && _MSC_VER <= 1200
// msvc 'for' scoping hack

/**
 *	@brief a simple raster image with fixed RGBA8 storage
 *
 *	The image data are always RGBA8. Alpha is stored in the most significant byte,
 *	followed by red, green and blue with decreasing significance.
 *
 *	The storage is very simple, each 32 bits in the buffer contains a single pixel,
 *	the first pixel is in top left corner, there is no scanline padding.
 */
struct TBmp {
	uint8_t n_former_bpc; /**< @brief former bits per channel, before conversion to RGBA8 */
	bool b_grayscale; /**< @brief grayscale flag (if set, the bitmap is assumed
		to contain grayscale image, stored as RGBA8) */
	bool b_alpha; /**< @brief alpha channel flag (if set, the alpha channel is significant;
		otherwise it's expected to be 0xff in all image pixels) */
	int n_width; /**< @brief image width, in pixels */
	int n_height; /**< @brief image height, in pixels */
	uint32_t *p_buffer; /**< @brief pointer to image data */

	/**
	 *	@brief deletes data buffer
	 */
	inline void Free()
	{
		delete[] p_buffer;
	}

	/**
	 *	@brief deletes data buffer and this
	 */
	inline void Delete()
	{
		delete[] p_buffer;
		delete this;
	}

	/**
	 *	@brief gets pointer to the first pixel of the bitmap, for use in point filters
	 *	@return Returns pointer to the first pixel of the bitmap.
	 */
	inline uint32_t *p_Begin()
	{
		return p_buffer;
	}

	/**
	 *	@brief gets pointer to the first pixel of the bitmap, for use in point filters
	 *	@return Returns const pointer to the first pixel of the bitmap.
	 */
	inline const uint32_t *p_Begin() const
	{
		return p_buffer;
	}

	/**
	 *	@brief gets pointer to one past the last pixel of the bitmap, for use in point filters
	 *	@return Returns pointer to one past the last pixel of the bitmap.
	 */
	inline uint32_t *p_End()
	{
		return p_buffer + (size_t(n_width) * n_height);
	}

	/**
	 *	@brief gets pointer to one past the last pixel of the bitmap, for use in point filters
	 *	@return Returns const pointer to one past the last pixel of the bitmap.
	 */
	inline const uint32_t *p_End() const
	{
		return p_buffer + (size_t(n_width) * n_height);
	}

	/**
	 *	@brief allocates a new bitmap with the specified parameters
	 *
	 *	@param[in] _n_width is width of the bitmap, in pixels
	 *	@param[in] _n_height is height of the bitmap, in pixels
	 *	@param[in] _b_grayscale is grayscale flag
	 *	@param[in] _b_alpha is alpha channel flag
	 *	@param[in] _n_former_bpc is number of bits of the intended contents
	 *		of the bitmap, before conversion to RGBA8
	 *
	 *	@return Returns pointer to the new bitmap on success, or 0 on failure.
	 */
	static TBmp *p_Alloc(int _n_width, int _n_height,
		bool _b_grayscale = false, bool _b_alpha = false, int _n_former_bpc = 8)
	{
		_ASSERTE(_n_width >= 0 && _n_height >= 0);
		TBmp *p_bmp;
		if(!(p_bmp = new(std::nothrow) TBmp))
			return 0;
		if(!(p_bmp->p_buffer = new(std::nothrow) uint32_t[size_t(_n_width) * _n_height])) {
			delete p_bmp;
			return 0;
		}
		p_bmp->n_width = _n_width;
		p_bmp->n_height = _n_height;
		p_bmp->b_grayscale = _b_grayscale;
		p_bmp->b_alpha = _b_alpha;
		p_bmp->n_former_bpc = _n_former_bpc;
		return p_bmp;
	}

	/**
	 *	@copydoc p_Alloc
	 *	@deprecated This is deprecated in favor of p_Alloc(), the effect is the same.
	 */
	static inline TBmp *p_CreateBitmap(int _n_width, int _n_height,
		bool _b_grayscale = false, bool _b_alpha = false, int _n_former_bpc = 8)
	{
		return p_Alloc(_n_width, _n_height, _b_grayscale, _b_alpha, _n_former_bpc);
	}

	/**
	 *	@brief creates a clone of the image
	 *	@param[in] b_ignore_contents is ignore contents flag (if set, only bitmap with
	 *		the same parameters is created, but the raster data are not copied)
	 *	@return Returns pointer to the new bitmap on success, or 0 on failure.
	 */
	TBmp *p_Clone(bool b_ignore_contents = false) const
	{
		TBmp *p_clone = p_Alloc(n_width, n_height, b_grayscale, b_alpha, n_former_bpc);
		if(p_buffer && p_clone && !b_ignore_contents)
			memcpy(p_clone->p_buffer, p_buffer, size_t(n_width) * n_height * sizeof(uint32_t));
		return p_clone;
	}

	/**
	 *	@brief fills the bitmap with constant color
	 *	@param[in] n_color is the fill color
	 */
	void Clear(uint32_t n_color)
	{
		std::fill(p_Begin(), p_End(), n_color);
		/*for(uint32_t *p_dest = p_buffer, *p_end = p_buffer + size_t(n_width) * n_height; p_dest != p_end; ++ p_dest)
			*p_dest = n_color;*/
	}

	/**
	 *	@brief drops alpha channel (makes the image opaque)
	 *	@note This has no effect if b_alpha is not set.
	 */
	void Drop_Alpha()
	{
		if(!b_alpha)
			return;
		// that was easy ...

		std::for_each(p_Begin(), p_End(), TMakeOpaque());
		b_alpha = false;
	}

	/**
	 *	@brief converts the bitmap to grayscale
	 *	@note This has no effect if the b_grayscale flag is already set.
	 */
	void Make_Grayscale()
	{
		if(b_grayscale)
			return;
		// that was easy ...

		/*for(uint32_t *p_pixel = p_buffer, *p_end = p_buffer + (size_t(n_width) * n_height); p_pixel != p_end; ++ p_pixel) {
			uint32_t c = *p_pixel;
			int r = (c >> 16) & 0xff;
			int g = (c >> 8) & 0xff;
			int b = c & 0xff;
			int n_grey = (r * int(.299f * 0x10000) + g * int(.578f * 0x10000) + b * int(.114f * 0x10000)) >> 16;
			*p_pixel = 0xff000000U | (n_grey << 16) | (n_grey << 8) | n_grey;
		}*/
		std::for_each(p_Begin(), p_End(), TGrayscaleConv());
		b_grayscale = true;
	}

	/**
	 *	@brief swaps between RGB and BGR channel order
	 *	@note The default order is ARGB, where A is in MSB and B is in LSB.
	 */
	void Swap_RGB_to_BGR()
	{
		/*for(uint32_t *p_dest = p_buffer, *p_end = p_buffer + size_t(n_width) * n_height; p_dest != p_end; ++ p_dest) {
			uint32_t n_color = *p_dest;
			*p_dest = (n_color & 0xff00ff00U) | ((n_color & 0xff0000) >> 16) | ((n_color & 0x0000ff) << 16);
		}*/
		std::for_each(p_Begin(), p_End(), TRGB_BGR_Swap());
	}

	/**
	 *	@brief flips the image
	 *	@param[in] b_vertical is flip direction (if set, flips vertical, if cleared, horizontal)
	 */
	void Flip(bool b_vertical)
	{
		if(b_vertical) {
			const int h = n_height, n_half_h = n_height / 2, w = n_width;
			uint32_t *p_top_scanline = p_buffer, *p_bottom_scanline =
				p_buffer + (h - 1) * w;
			for(int y = 0; y < n_half_h; ++ y, p_top_scanline += w, p_bottom_scanline -= w) {
				for(int x = 0; x < w; ++ x)
					std::swap(p_top_scanline[x], p_bottom_scanline[x]);
			}
		} else {
			const int h = n_height, n_half_w = n_width / 2, w = n_width;
			uint32_t *p_left = p_buffer, *p_right = p_buffer + (w - 1);
			int y;
			for(y = 0; y < h - 1; y += 2, p_left += w, p_right += w) {
				for(int x = 0; x < n_half_w; ++ x, ++ p_left, -- p_right)
					std::swap(*p_left, *p_right);
				// do one line, shifting pointers towards the center of the scanline

				p_left += w;
				p_right += w;
				// shift to the next line

				for(int x = 0; x < n_half_w; ++ x, -- p_left, ++ p_right)
					std::swap(*p_left, *p_right);
				// do the other line, shifting the pointers back to the edges
			}
			if(y < h) {
				for(int x = 0; x < n_half_w; ++ x, ++ p_left, -- p_right)
					std::swap(*p_left, *p_right);
				// do the last (even) line
			}
		}
	}

	/**
	 *	@brief transposes the image (exchanges rows with columns)
	 *	@return Returns true on success, false on failure (not enough memory for the temporary buffer).
	 *	@note This does not change the pointer to the image data.
	 */
	bool Transpose()
	{
		const int h = n_height, w = n_width;
		if(w == h) {
			for(int x = 0; x < w; ++ x) {
				for(int y = 0; y < x; ++ y)
					std::swap(p_buffer[x + w * y], p_buffer[y + w * x]);
			}
			// inplace transpose for square images
		} else {
			TBmp *p_temp;
			if(!(p_temp = p_Clone())) // an extra copy in exchange for not changing the image buffer
				return false;

			std::swap(n_width, n_height);
			const uint32_t *p_src = p_temp->p_buffer; // antialiass
			for(int x = 0; x < w; ++ x) {
				for(int y = 0; y < h; ++ y)
					p_buffer[y + h * x] = p_src[x + w * y];
			}
			// copy the image back transposed

			p_temp->Delete();
		}

		return true;
	}

	/**
	 *	@brief turns the image
	 *	@param[in] n_multiples_pi_half_cw is angle in multiples of pi/2 (or 90°), clockwise
	 *	@return Returns the rotated image on success, 0 on failure (not enough memory).
	 */
	TBmp *p_Turn(int n_multiples_pi_half_cw) const
	{
		int n_angle = n_multiples_pi_half_cw & 3;
		// only four possible angles; make them positive

		if(!n_angle)
			return p_Clone(); // zero angle - just clone
		else if(n_angle == 2) { // rotate 180, same as flip horizontal and vertical
			TBmp *p_rotate = p_Clone();
			if(!p_rotate)
				return 0;
			p_rotate->Flip(false);
			p_rotate->Flip(true); // note that this is not optimal, a lot of data movement // todo
			return p_rotate;
		}
		// handle simple cases

		TBmp *p_rotate = p_Clone(false);
		if(!p_rotate)
			return 0;
		std::swap(p_rotate->n_height, p_rotate->n_width);
		// alloc bitmap of the same shape

		const int sw = n_width, sh = n_height;
		if(n_angle == 1) { // rotate 90 cw
			for(int sy = 0; sy < sh; ++ sy) {
				int dx = sh - 1 - sy;
				for(int sx = 0; sx < sw; ++ sx) {
					int dy = sx;
					p_rotate->p_buffer[dx + sh * dy] = p_buffer[sx + sw * sy];
				}
			}
			// x becomes y
			// y becomes h - x
		} else /*if(n_angle == 3)*/ { // rotate 270 cw
			_ASSERTE(n_angle == 3);
			for(int sy = 0; sy < sh; ++ sy) {
				int dx = sy;
				for(int sx = 0; sx < sw; ++ sx) {
					int dy = sw - 1 - sx;
					p_rotate->p_buffer[dx + sh * dy] = p_buffer[sx + sw * sy];
				}
			}
			// x becomes w - y
			// y becomes x
		}

		return p_rotate;
	}

	/**
	 *	@brief crops the image
	 *
	 *	@param[in] x is a coordinate of top-left corner of the crop rectangle
	 *	@param[in] y is a coordinate of top-left corner of the crop rectangle
	 *	@param[in] _n_width is width of the crop rectangle, in pixels
	 *	@param[in] _n_height is height of the crop rectangle, in pixels
	 *
	 *	@return Returns pointer to the new bitmap on success, or 0 on failure.
	 *
	 *	@note The cropping rectangle must lie completely inside the bitmap.
	 */
	TBmp *p_Crop(int x, int y, int _n_width, int _n_height) const
	{
		_ASSERTE(x >= 0 && y >= 0 && x + _n_width <= n_width && y + _n_height <= n_height);

		TBmp *p_clone = p_Alloc(_n_width, _n_height, b_grayscale, b_alpha, n_former_bpc); // not quite a clone - has a different size
		if(p_buffer && p_clone) {
			for(int yy = 0; yy < _n_height; ++ yy) {
				memcpy(p_clone->p_buffer + _n_width * yy,
					p_buffer + (x + (y + yy) * n_width), _n_width * sizeof(uint32_t));
			}
		}
		return p_clone;
	}

	/**
	 *	@brief general "canvas size" operation
	 *
	 *	@param[in] x is a coordinate of top-left corner of the crop rectangle in the source image
	 *	@param[in] y is a coordinate of top-left corner of the crop rectangle in the source image
	 *	@param[in] _n_width is width of the crop rectangle, in pixels
	 *	@param[in] _n_height is height of the crop rectangle, in pixels
	 *	@param[in] n_dest_x is a coordinate of top-left corner of the crop rectangle in the destination image
	 *	@param[in] n_dest_y is a coordinate of top-left corner of the crop rectangle in the destination image
	 *	@param[in] n_background_color is color of unspecified pixels if the crop rectangle is bigger
	 *
	 *	@return Returns pointer to the new bitmap on success, or 0 on failure.
	 *
	 *	@note The cropping rectangle is arbitrary.
	 *
	 *	@todo This is practically untested, debug this.
	 */
	TBmp *p_Crop(int x, int y, int _n_width, int _n_height,
		int n_dest_x, int n_dest_y, uint32_t n_background_color) const
	{
		int n_src_width = n_width;
		int n_src_height = n_height;
		if(n_dest_x < 0) {
			x -= n_dest_x;
			n_src_width += n_dest_x;
			n_dest_x = 0;
		}
		if(n_dest_y < 0) {
			y -= n_dest_y;
			n_src_height += n_dest_y;
			n_dest_y = 0;
		}
		// destination x and y can be negative; fix it like this

		TBmp *p_clone = p_Alloc(_n_width, _n_height, b_grayscale, b_alpha, n_former_bpc); // not quite a clone - has a different size
		if(p_buffer && p_clone) {
			//p_clone->Clear(n_background_color);
			// sub-optimal, much of the background will be typically overwritten

			int n_fill_height = min(_n_height - n_dest_y, n_src_height); // smaller one
			int n_fill_width = min(_n_width - n_dest_x, n_src_width); // smaller one
			if(n_fill_width < 0 || n_fill_height < 0) {
				p_clone->Clear(n_background_color); // nothing of the original iamge is seen
				return p_clone;
			}
			// calculate how much the images intersect

			if(n_dest_y > 0) {
				uint32_t *p_dest = p_clone->p_buffer;
				for(int yy = 0; yy < n_dest_y; ++ yy)
					for(int xx = 0; xx < _n_width; ++ xx, ++ p_dest)
						*p_dest = n_background_color;
				_ASSERTE(p_dest <= p_clone->p_buffer + _n_width * _n_height);
			}
			// fill upper half

			{
				uint32_t *p_dest = p_clone->p_buffer + _n_width * n_dest_y;
				for(int yy = n_dest_y; yy < n_dest_y + n_fill_height; ++ yy, p_dest += _n_width) {
					for(int xx = 0; xx < n_dest_x; ++ xx)
						p_dest[xx] = n_background_color;
					// fill before

					memcpy(p_dest + n_dest_x, p_buffer + (x + (y + yy - n_dest_y) * n_width),
						n_fill_width * sizeof(uint32_t));
					// copy from the source image

					for(int xx = n_dest_x + n_fill_width; xx < _n_width; ++ xx)
						p_dest[xx] = n_background_color;
					// fill after
				}
				_ASSERTE(p_dest <= p_clone->p_buffer + _n_width * _n_height);
			}

			if(n_dest_y + n_fill_height < _n_height) {
				uint32_t *p_dest = p_clone->p_buffer + _n_width * (n_dest_y + n_fill_height);
				_ASSERTE(p_dest < p_clone->p_buffer + _n_width * _n_height);
				for(int yy = n_dest_y + n_fill_height; yy < _n_height; ++ yy)
					for(int xx = 0; xx < _n_width; ++ xx, ++ p_dest)
						*p_dest = n_background_color;
				_ASSERTE(p_dest <= p_clone->p_buffer + _n_width * _n_height);
			}
			// fill lower half
		}
		return p_clone;
	}

	// todo - write a copy function that works like BitBlt()

	/**
	 *	@brief filter types for p_Upscale()
	 */
	enum {
		filter_Nearest, /**< @brief nearest neighbor filter */
		filter_Bilinear, /**< @brief bilinear filter */
		filter_Bicubic /**< @brief bicubic filter */
	};

	/**
	 *	@brief magnifies the image
	 *
	 *	@param[in] n_new_width is new width, in pixels
	 *	@param[in] n_new_height is new height, in pixels
	 *	@param[in] n_filter_type is one of filter_*, currently only filter_Nearest is supported
	 *
	 *	@return Returns the upscaled image on success, 0 on failure (not enough memory).
	 *
	 *	@note The dimensions do not generally need to be greater than the original image, but if
	 *		they are lower, aliassing may occur and it is not a goal of this function to avoid it.
	 */
	TBmp *p_Upscale(int n_new_width, int n_new_height, int n_filter_type = filter_Bilinear) const
	{
		_ASSERTE(n_filter_type == filter_Nearest/* ||
			n_filter_type == filter_Bilinear || n_filter_type == filter_Bicubic*/);  // others not implemented yet

		TBmp *p_scaled;
		if(!(p_scaled = p_Alloc(n_new_width, n_new_height, b_grayscale, b_alpha, n_former_bpc)))
			return 0;
		// alloc a new bitmap

		if(n_new_width >= n_width && n_new_height >= n_height &&
		   n_new_width / n_width == n_new_height / n_height &&
		   n_new_width % n_width == 0 && n_new_height % n_height == 0) {
			int n_factor = n_new_width / n_width;
			// scaling by an integer factor

			if(n_filter_type == filter_Nearest) {
				const uint32_t *p_src = p_buffer;
				uint32_t *const p_dest = p_scaled->p_buffer; // do not change address of dest
				for(int y = 0, w = n_width, h = n_height; y < h; ++ y) {
					for(int x = 0; x < w; ++ x, ++ p_src) {
						uint32_t n_src = *p_src;
						for(int dy = 0; dy < n_factor; ++ dy) {
							for(int dx = 0; dx < n_factor; ++ dx)
								p_dest[(x * n_factor + dx) + (y * n_factor + dy) * n_new_width] = n_src;
						}
					}
				}
				// could handle power-of-two factors with a shift instead of mul
			} else {
				// todo - implement other filters (precalc weights for the neighbors in a LUT,
				// do the same loop as for nearest, handle border cases separately)
			}
		} else {
			// general scaling

			const uint32_t *const p_src = p_buffer; // do not change address of src
			uint32_t *p_dest = p_scaled->p_buffer;
			if(n_filter_type == filter_Nearest) {
				for(int y = 0, w = n_width, h = n_height; y < n_new_height; ++ y) {
					int sy = (y * h) / n_new_height; // the product is up to n_height * (n_new_height - 1), might need int64 for that
					for(int x = 0; x < n_new_width; ++ x, ++ p_dest) {
						int sx = (x * w) / n_new_width; // the product is up to n_width * (n_new_width - 1), might need int64 for that
						*p_dest = p_src[sx + sy * w];
					}
				}
			} else {
				// todo - implement other filters (calculate how big is the period for the LUTs, if under thresh
				// for both x and y, precalc and do blocked processing, otherwise do the same as for nearest)
			}
		}

		return p_scaled;
	}

	/**
	 *	@brief fills a selected pixel with a given color
	 *
	 *	@param[in] x is a coordinate of the pixel to fill
	 *	@param[in] y is a coordinate of the pixel to fill
	 *	@param[in] n_color is the fill color
	 *
	 *	@note This performs array boundary checking,
	 *		coordinates outside the bitmap are ok.
	 */
	inline void PutPixel(int x, int y, uint32_t n_color)
	{
		if(x >= 0 && x < n_width && y >= 0 && y < n_height)
			p_buffer[int(x) + n_width * int(y)] = n_color;
	}

	/**
	 *	@brief fills a selected pixel with a given color, with subpixel precision
	 *
	 *	@param[in] x is a coordinate of the pixel to fill
	 *	@param[in] y is a coordinate of the pixel to fill
	 *	@param[in] n_color is the fill color
	 *
	 *	@note This actually fills up to 4 pixels, depending on the fractional coordinates.
	 *	@note This performs array boundary checking,
	 *		coordinates outside the bitmap are ok.
	 */
	inline void PutPixel_AA(float f_x, float f_y, uint32_t n_color)
	{
		int x = int(floor(f_x)), y = int(floor(f_y));
		float f_frac_x = f_x - x;
		float f_frac_y = f_y - y;
		int n_alpha = (n_color >> 24) & 0xff;
		int n_weight_00 = int(n_alpha * (1 - f_frac_x) * (1 - f_frac_y));
		int n_weight_10 = int(n_alpha * f_frac_x * (1 - f_frac_y));
		int n_weight_01 = int(n_alpha * (1 - f_frac_x) * f_frac_y);
		int n_weight_11 = int(n_alpha * f_frac_x * f_frac_y);
		if(x >= 0 && x < n_width && y >= 0 && y < n_height)
			AlphaBlend(p_buffer[x + n_width * y], n_color, n_weight_00);
		if(x + 1 >= 0 && x + 1 < n_width && y >= 0 && y < n_height)
			AlphaBlend(p_buffer[(x + 1) + n_width * y], n_color, n_weight_10);
		if(x >= 0 && x < n_width && y + 1 >= 0 && y + 1 < n_height)
			AlphaBlend(p_buffer[x + n_width * (y + 1)], n_color, n_weight_01);
		if(x + 1 >= 0 && x + 1 < n_width && y + 1 >= 0 && y + 1 < n_height)
			AlphaBlend(p_buffer[(x + 1) + n_width * (y + 1)], n_color, n_weight_11);
	}

	/**
	 *	@brief draws an axis aligned rectangle (only lines, no fill)
	 *
	 *	@param[in] n_x0 is a coordinate of the top-left corner
	 *	@param[in] n_y0 is a coordinate of the top-left corner
	 *	@param[in] n_x1 is a coordinate of the bottom-right corner
	 *	@param[in] n_y1 is a coordinate of the bottom-right corner
	 *	@param[in] n_color is the line color
	 *	@param[in] n_line_width is line width, in pixels
	 */
	void DrawRect(int n_x0, int n_y0, int n_x1, int n_y1, uint32_t n_color, int n_line_width = 1)
	{
		if(n_x0 > n_x1)
			std::swap(n_x0, n_x1);
		if(n_y0 > n_y1)
			std::swap(n_y0, n_y1);
		// make sure it is ordered

		int n_inner_line_width = n_line_width / 2;
		n_x0 += n_inner_line_width;
		n_y0 += n_inner_line_width;
		n_x1 -= n_inner_line_width;
		n_y1 -= n_inner_line_width;
		// make a smaller rectangle

		for(int l = 0; l < n_line_width; ++ l, -- n_x0, -- n_y0, ++ n_x1, ++ n_y1) {
			if(n_y1 > n_y0) {
				if(n_y0 >= 0 && n_y0 < n_height) {
					_ASSERTE(n_y1 >= 0);
					if(n_y1 < n_height) {
						for(int x = max(0, n_x0); x < min(n_width, n_x1 + 1); ++ x) {
							p_buffer[x + n_y0 * n_width] = n_color;
							p_buffer[x + n_y1 * n_width] = n_color;
						}
						// both are in
					} else {
						for(int x = max(0, n_x0); x < min(n_width, n_x1 + 1); ++ x)
							p_buffer[x + n_y0 * n_width] = n_color;
					}
				} else if(n_y1 >= 0 && n_y1 < n_height) {
					for(int x = max(0, n_x0); x < min(n_width, n_x1 + 1); ++ x)
						p_buffer[x + n_y1 * n_width] = n_color;
				}
			}
			// draw horizontal lines

			if(n_x1 > n_x0) {
				if(n_x0 >= 0 && n_x0 < n_width) {
					_ASSERTE(n_x1 >= 0);
					if(n_x1 < n_width) {
						for(int y = max(0, n_y0); y < min(n_height, n_y1 + 1); ++ y) {
							p_buffer[n_x0 + y * n_width] = n_color;
							p_buffer[n_x1 + y * n_width] = n_color;
						}
						// both are in
					} else {
						for(int y = max(0, n_y0); y < min(n_height, n_y1 + 1); ++ y)
							p_buffer[n_x0 + y * n_width] = n_color;
					}
				} else if(n_x1 >= 0 && n_x1 < n_width) {
					for(int y = max(0, n_y0); y < min(n_height, n_y1 + 1); ++ y)
						p_buffer[n_x1 + y * n_width] = n_color;
				}
			}
			// draw vertical lines
		}
		// each loop iteration draws thickness 1 rectangle, and expands the dimensions
	}

	/**
	 *	@brief fills an axis aligned rectangle with constant color
	 *
	 *	@param[in] n_x0 is a coordinate of the top-left corner
	 *	@param[in] n_y0 is a coordinate of the top-left corner
	 *	@param[in] n_x1 is a coordinate of the bottom-right corner
	 *	@param[in] n_y1 is a coordinate of the bottom-right corner
	 *	@param[in] n_color is the fill color
	 */
	void FillRect(int n_x0, int n_y0, int n_x1, int n_y1, uint32_t n_color)
	{
		if(n_x0 > n_x1)
			std::swap(n_x0, n_x1);
		if(n_y0 > n_y1)
			std::swap(n_y0, n_y1);
		// make sure it is ordered

		if(n_x1 < 0 || n_y1 < 0)
			return;
		if(n_x0 >= n_width || n_y0 >= n_height)
			return;
		// simple rejection

		n_x0 = max(0, n_x0);
		n_y0 = max(0, n_y0);
		n_x1 = min(n_width, n_x1 + 1) - n_x0; // number of pixels to fill
		_ASSERTE(n_x1 >= 0);
		_ASSERTE(n_x0 + n_x1 <= n_width);
		n_y1 = min(n_height, n_y1 + 1); // y one past the last scanline
		// make sure it is inside

		uint32_t *p_scan = p_buffer + n_x0 + n_y0 * n_width;
		for(int y = n_y0; y < n_y1; ++ y, p_scan += n_width) {
			for(uint32_t *p_ptr = p_scan, *p_end = p_scan + n_x1; p_ptr != p_end; ++ p_ptr)
				*p_ptr = n_color;
		}
		// fill
	}

	/**
	 *	@brief simple implementation of flood-fill
	 *
	 *	@param[in] n_seed_x is horizontal position of seed (must be inside the image)
	 *	@param[in] n_seed_y is vertical position of seed (must be inside the image)
	 *	@param[in] n_fill_color is color to fill with (if the color of the
	 *		seed pixel equals this color, the function immediately succeeds)
	 *	@param[in] n_connectivity is pixel connectivity (4 or 8)
	 *
	 *	@return Returns true on success, false on failure (not enough memory for backtracking).
	 */
	bool FloodFill(int n_seed_x, int n_seed_y, uint32_t n_fill_color, int n_connectivity = 4)
	{
		_ASSERTE(n_connectivity == 4 || n_connectivity == 8);
		_ASSERTE(n_seed_x >= 0 && n_seed_x < n_width);
		_ASSERTE(n_seed_y >= 0 && n_seed_y < n_height);

		uint32_t n_bk_color = p_buffer[n_seed_x + n_seed_y * n_width];
		if(n_bk_color == n_fill_color)
			return true; // already filled / would loop indefinitely
		// get background color

		try {
			std::vector<std::pair<int, int> > fill_path;
			fill_path.push_back(std::make_pair(n_seed_x, n_seed_y));
			// add the seed to the stack

			while(!fill_path.empty()) {
				std::pair<int, int> pt = fill_path.back();
				int x = pt.first, y = pt.second;
				// get position from the stack

				p_buffer[x + y * n_width] = n_fill_color;
				// mark this location as visited

				if(x + 1 < n_width && p_buffer[x + 1 + y * n_width] == n_bk_color)
					fill_path.push_back(std::make_pair(x + 1, y));
				else if(x > 0 && p_buffer[x - 1 + y * n_width] == n_bk_color)
					fill_path.push_back(std::make_pair(x - 1, y));
				else if(y > 0 && p_buffer[x + (y - 1) * n_width] == n_bk_color)
					fill_path.push_back(std::make_pair(x, y - 1));
				else if(y + 1 < n_height && p_buffer[x + (y + 1) * n_width] == n_bk_color)
					fill_path.push_back(std::make_pair(x, y + 1));
				else {
					if(n_connectivity == 4)
						fill_path.erase(fill_path.end() - 1);
					else if(x + 1 < n_width && y > 0 && p_buffer[x + 1 + (y - 1) * n_width] == n_bk_color)
						fill_path.push_back(std::make_pair(x + 1, y - 1));
					else if(x + 1 < n_width && y + 1 < n_height && p_buffer[x + 1 + (y + 1) * n_width] == n_bk_color)
						fill_path.push_back(std::make_pair(x + 1, y + 1));
					else if(x > 0 && y > 0 && p_buffer[x - 1 + (y - 1) * n_width] == n_bk_color)
						fill_path.push_back(std::make_pair(x - 1, y - 1));
					else if(x > 0 && y + 1 < n_height && p_buffer[x - 1 + (y + 1) * n_width] == n_bk_color)
						fill_path.push_back(std::make_pair(x - 1, y + 1));
					else
						fill_path.erase(fill_path.end() - 1);
					// 8-connectivity
				}
				// try to visit other unvisited adjacent positions
			}
			// use flood fill (naive implementation)
		} catch(std::bad_alloc&) {
			return false; // out of memory
		}

		return true;
	}

	/**
	 *	@brief clips a line to bitmap interior
	 *
	 *	@param[in,out] r_f_x0 is a coordinate of the first line point
	 *	@param[in,out] r_f_y0 is a coordinate of the first line point
	 *	@param[in,out] r_f_x1 is a coordinate of the second line point
	 *	@param[in,out] r_f_y1 is a coordinate of the second line point
	 *
	 *	@return Returns true if the line is inside, false if it is completely
	 *		outside (early reject, values of arguments are not changed).
	 */
	inline bool ClipLine(float &r_f_x0, float &r_f_y0, float &r_f_x1, float &r_f_y1) const
	{
		if(r_f_x0 != r_f_x0 || r_f_y0 != r_f_y0 || r_f_x1 != r_f_x1 || r_f_y1 != r_f_y1)
			return false;
		// handle NaNs

		bool b_not_narrow;
		float f_dxdy = ((b_not_narrow = (fabs(r_f_x1 - r_f_x0) > 1e-5f)))?
			(r_f_y1 - r_f_y0) / (r_f_x1 - r_f_x0) : 0;
		if(r_f_x0 < 0 || r_f_x1 < 0) {
			if(r_f_x0 < 0 && r_f_x1 < 0)
				return false; // offscreen
			if(r_f_x0 < 0) {
				r_f_y0 -= f_dxdy * r_f_x0; // note this rounds ...
				r_f_x0 = 0;
			} else {
				r_f_y1 -= f_dxdy * r_f_x1; // note this rounds ...
				r_f_x1 = 0;
			}
		}
		const int n_w_max = n_width - 1;
		if(r_f_x0 > n_w_max || r_f_x1 > n_w_max) {
			if(r_f_x0 > n_w_max && r_f_x1 > n_w_max)
				return false; // offscreen
			if(r_f_x0 > n_w_max) {
				float dx = r_f_x0 - n_w_max;
				r_f_y0 -= f_dxdy * dx; // note this rounds ...
				r_f_x0 = float(n_w_max);
			} else {
				float dx = r_f_x1 - n_w_max;
				r_f_y1 -= f_dxdy * dx; // note this rounds ...
				r_f_x1 = float(n_w_max);
			}
		}
		if(!b_not_narrow)
			f_dxdy = 1e37f; // stable value for this part (or could branch below)
		if(r_f_y0 < 0 || r_f_y1 < 0) {
			if(r_f_y0 < 0 && r_f_y1 < 0)
				return false; // offscreen
			if(r_f_y0 < 0) {
				r_f_x0 -= r_f_y0 / f_dxdy; // note this rounds ...
				r_f_y0 = 0;
			} else {
				r_f_x1 -= r_f_y1 / f_dxdy; // note this rounds ...
				r_f_y1 = 0;
			}
		}
		const int n_h_max = n_height - 1;
		if(r_f_y0 > n_h_max || r_f_y1 > n_h_max) {
			if(r_f_y0 > n_h_max && r_f_y1 > n_h_max)
				return false; // offscreen
			if(r_f_y0 > n_h_max) {
				float dy = r_f_y0 - n_h_max;
				r_f_x0 -= dy / f_dxdy; // note this rounds ...
				r_f_y0 = float(n_h_max);
			} else {
				float dy = r_f_y1 - n_h_max;
				r_f_x1 -= dy / f_dxdy; // note this rounds ...
				r_f_y1 = float(n_h_max);
			}
		}
		// perform simple clipping

		_ASSERTE(int(r_f_x0) >= 0 && int(r_f_x0) <= n_w_max);
		_ASSERTE(int(r_f_y0) >= 0 && int(r_f_y0) <= n_h_max);
		_ASSERTE(int(r_f_x1) >= 0 && int(r_f_x1) <= n_w_max);
		_ASSERTE(int(r_f_y1) >= 0 && int(r_f_y1) <= n_h_max);

		return true;
	}

	/**
	 *	@brief draws a solid-color line
	 *
	 *	@param[in] n_x0 is a coordinate of the first line point
	 *	@param[in] n_y0 is a coordinate of the first line point
	 *	@param[in] n_x1 is a coordinate of the second line point
	 *	@param[in] n_y1 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] n_line_width is the line width, in pixels
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 */
	void DrawLine(int n_x0, int n_y0, int n_x1, int n_y1, uint32_t n_color, int n_line_width = 1)
	{
		if(n_line_width <= 0)
			return;
		// too thin

		float f_x0 = float(n_x0) + .5f, f_y0 = float(n_y0) + .5f;
		float f_x1 = float(n_x1) + .5f, f_y1 = float(n_y1) + .5f;
		// adjust lines with integer coordinates to be on pixel centers (makes tilted lines look better)

		DrawLine_SP(f_x0, f_y0, f_x1, f_y1, n_color, n_line_width);
		// use fancy function
	}

	/**
	 *	@brief draws a solid-color line with subpixel precision
	 *
	 *	@param[in] f_x0 is a coordinate of the first line point
	 *	@param[in] f_y0 is a coordinate of the first line point
	 *	@param[in] f_x1 is a coordinate of the second line point
	 *	@param[in] f_y1 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] n_line_width is the line width, in pixels
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 */
	void DrawLine_SP(float f_x0, float f_y0, float f_x1, float f_y1,
		uint32_t n_color, int n_line_width = 1)
	{
		if(n_line_width <= 0)
			return;
		// too thin

		if(!ClipLine(f_x0, f_y0, f_x1, f_y1))
			return;
		// perform simple clipping

		/*if(f_x0 >= 0 && f_x0 < n_width && f_y0 >= 0 && f_y0 < n_height)
			p_buffer[int(f_x0) + n_width * int(f_y0)] = 0xffff00ff;
		if(f_x1 >= 0 && f_x1 < n_width && f_y1 >= 0 && f_y1 < n_height)
			p_buffer[int(f_x1) + n_width * int(f_y1)] = 0xffff00ff;*/
		// debug - mark endpoints // now it fails olny on very short lines (the test with the spiral)

		_ASSERTE(max(abs(int(f_x0) - int(f_x1)), abs(int(f_y0) - int(f_y1))) <=
			max(n_width, n_height)); // line lenght is now bound by bitmap size
		bool b_steep = fabs(f_y0 - f_y1) > fabs(f_x0 - f_x1);
		if(b_steep) {
			std::swap(f_x0, f_y0);
			std::swap(f_x1, f_y1);
			// makes sure it is rasterized in the larger dimension
		}
		if(f_x0 > f_x1) {
			std::swap(f_x0, f_x1);
			std::swap(f_y0, f_y1);
		}
		if(n_line_width == 1) {
			int n_len = abs(int(floor(f_x1) - floor(f_x0)));
            CInterpolator lerp(int(f_y0 * 256), int(f_y1 * 256), n_len);
			// note this is not adjusted for fractional x

			_ASSERTE(f_x0 <= f_x1);
			if(b_steep) {
				for(int n_y = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_y < n_end; ++ n_y, ++ lerp) {
					int n_x = lerp.y() >> 8;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
				}
			} else {
				for(int n_x = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_x < n_end; ++ n_x, ++ lerp) {
					int n_y = lerp.y() >> 8;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
				}
			}
			// thin lines
		} else {
			float f_dx = fabs(f_x1 - f_x0);
			float f_dy = fabs(f_y1 - f_y0);
			float f_thickness_scale = sqrt(f_dx * f_dx + f_dy * f_dy) / max(1.0f, max(f_dx, f_dy));
			n_line_width = max(n_line_width, int(n_line_width * f_thickness_scale + .5f));
			// adjust line thickness based on line angle

			int n_line_extent_top = (n_line_width - 1) / 2;
			int n_line_extent_bottom = (n_line_width - 1) - n_line_extent_top;
			// calculate extent on top and bottom

			int n_len = abs(int(floor(f_x1) - floor(f_x0)));
            CInterpolator lerp(int(f_y0 * 256), int(f_y1 * 256), n_len);
			// note this is not adjusted for fractional n_x

			if(b_steep) {
				for(int n_y = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_y < n_end; ++ n_y, ++ lerp) {
					int n_x = lerp.y() >> 8;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
					for(int dy = 1; dy <= n_line_extent_top; ++ dy) {
						if(n_x + dy < n_width)
							p_buffer[n_x + dy + n_y * n_width] = n_color;
					}
					for(int dy = 1; dy <= n_line_extent_bottom; ++ dy) {
						if(n_x >= dy)
							p_buffer[n_x - dy + n_y * n_width] = n_color;
					}
				}
			} else {
				for(int n_x = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_x < n_end; ++ n_x, ++ lerp) {
					int n_y = lerp.y() >> 8;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
					for(int dy = 1; dy <= n_line_extent_top; ++ dy) {
						if(n_y + dy < n_height)
							p_buffer[n_x + (n_y + dy) * n_width] = n_color;
					}
					for(int dy = 1; dy <= n_line_extent_bottom; ++ dy) {
						if(n_y >= dy)
							p_buffer[n_x + (n_y - dy) * n_width] = n_color;
					}
				}
			}
			// thick lines
		}

		/*if(b_steep) { // !!
			std::swap(f_x0, f_y0);
			std::swap(f_x1, f_y1);
		}
		if(f_x0 >= 0 && f_x0 < n_width && f_y0 >= 0 && f_y0 < n_height)
			if(p_buffer[int(f_x0) + n_width * int(f_y0)] != n_color) printf("start\n");
		if(f_x1 >= 0 && f_x1 < n_width && f_y1 >= 0 && f_y1 < n_height)
			if(p_buffer[int(f_x1) + n_width * int(f_y1)] != n_color) printf("end\n");*/
		// make sure that the endpoints were filled
	}

	/**
	 *	@brief draws a solid-color line with subpixel precision
	 *
	 *	@param[in] f_x0 is a coordinate of the first line point
	 *	@param[in] f_y0 is a coordinate of the first line point
	 *	@param[in] f_x1 is a coordinate of the second line point
	 *	@param[in] f_y1 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] f_line_width is the line width, in pixels
	 *	@param[in] n_line_end_type is line end type (0 = natural / fastest, 1 = bevel, 2 = round)
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 */
	void DrawLine_SP2(float f_x0, float f_y0, float f_x1, float f_y1,
		uint32_t n_color, float f_line_width = 1, int n_line_end_type = 0)
	{
		_ASSERTE(n_line_end_type >= 0 && n_line_end_type <= 2);
		if(f_line_width <= 0)
			return;
		// too thin

		if(!ClipLine(f_x0, f_y0, f_x1, f_y1))
			return;
		// perform simple clipping

		_ASSERTE(max(abs(int(f_x0) - int(f_x1)), abs(int(f_y0) - int(f_y1))) <=
			max(n_width, n_height)); // line lenght is now bound by bitmap size
		bool b_steep = fabs(f_y0 - f_y1) > fabs(f_x0 - f_x1);
		if(b_steep) {
			std::swap(f_x0, f_y0);
			std::swap(f_x1, f_y1);
			// makes sure it is rasterized in the larger dimension
		}
		if(f_x0 > f_x1) {
			std::swap(f_x0, f_x1);
			std::swap(f_y0, f_y1);
		}

		const int n_FP_factor = 1024;
		const int n_FP_shift = 10;
		// we use DDA here, no need to use 16:16 precision

		const float f_line_width_orig = f_line_width;
		const float f_dxdy = (fabs(f_x1 - f_x0) > 1e-5f)? (f_y1 - f_y0) / (f_x1 - f_x0) : 0;
		f_line_width = f_line_width * sqrt(1 + f_dxdy * f_dxdy);
		// update the line width, based on the slope of the line
		// note that this is corrected for b_steep

		if(f_line_width <= 1) {
			const int n_len = abs(int(floor(f_x1) - floor(f_x0)));
			//const int n_len = int(ceil(fabs(f_x1 - f_x0)));
#if 0
			float f_frac_x0 = (f_x0 - floor(f_x0));
			float f_frac_x1 = (f_x1 - floor(f_x1));
			_ASSERTE(fabs(f_dxdy) <= 1 && fabs(f_frac_x0) <= 1);
            CInterpolator lerp(int((f_y0) * n_FP_factor) + int((f_frac_x0 * f_dxdy) * n_FP_factor),
							   int((f_y1) * n_FP_factor) + int((f_frac_x1 * f_dxdy) * n_FP_factor), n_len); // sum in FP otherwise the endpoints will be off!
			/*CInterpolator y_fix(int((-f_frac_x0 * f_dxdy) * n_FP_factor),
				int((-f_frac_x1 * f_dxdy) * n_FP_factor), n_len);*/ // fix the first / last pixel displacement (removes the subpixel accuracy, no precision gained)
#endif
			CInterpolator lerp(int(f_y0 * n_FP_factor), int(f_y1 * n_FP_factor), n_len); // hits line ends perfectly
			// note that this is not adjusted for fractional x

			_ASSERTE(f_x0 <= f_x1);
			if(b_steep) {
				for(int n_y = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_y < n_end; ++ n_y, ++ lerp) {
					int n_x = lerp.y() >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
				}
			} else {
				for(int n_x = int(floor(f_x0)), n_end = int(floor(f_x1)) + 1; n_x < n_end; ++ n_x, ++ lerp) {
					int n_y = lerp.y() >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					p_buffer[n_x + n_y * n_width] = n_color;
				}
			}
			// thin lines, do not care about the endpoints
		} else {
			float f_line_extent = (f_line_width - 1) / 2;
			const int n_line_extent_top = int(n_FP_factor * f_line_extent);
			const int n_line_extent_bottom = n_line_extent_top; // just semantics, negligible error //int((n_FP_factor * (f_line_width - 1)) - n_line_extent_top);
			// calculate extent on top and bottom

			const float f_d0 = floor(f_x0) + floor(f_y0) * f_dxdy - .5f;
			const float f_d1 = (floor(f_x1) + 1) + floor(f_y1) * f_dxdy - .5f; // the half reduces abrupt swimming artefacts in lines with low angles
			// note that we need to use the integer coordinates.
			// using floating point coordinates makes the line not reach the ends / overreach them

			if(n_line_end_type == 2) {
				_ASSERTE(f_line_width_orig < INT_MAX / (n_FP_factor * n_FP_factor * 4)); // make sure we won't have fixed point overflows
				const int n_radius = int((f_line_width_orig - 1) / 2 * n_FP_factor);
				//const float f_radius = float(n_radius) / n_FP_factor; // to account for any roundoff in calculation of n_radius
				int n_x_center0 = int(f_x0 * n_FP_factor), n_y_center0 = int(f_y0 * n_FP_factor);
				int n_x_center1 = int(f_x1 * n_FP_factor), n_y_center1 = int(f_y1 * n_FP_factor);
				if(b_steep) {
					std::swap(n_x_center0, n_y_center0);
					std::swap(n_x_center1, n_y_center1);
				}
				int n_x = n_FP_factor / 2; // 0.5px
				//int n_y = int(sqrt(double(max(0, ((n_radius * n_radius) >> n_FP_shift) - n_FP_factor / 4)))) << (n_FP_shift / 2);
				int n_y = int(.5 + sqrt(double(max(0, n_radius * n_radius - n_FP_factor * n_FP_factor / 4))));
				//int n_y_ref = int(sqrt(max(.0f, f_radius * f_radius - .25f)) * n_FP_factor); // (r^2 - 0.5px^2)^0.5
				//_ASSERTE(abs(n_y - n_y_ref) <= 1);
				//int n_y = int(sqrt(double(max(0, ((n_radius * n_radius) >> n_FP_shift) - n_FP_factor / 4)))) << (n_FP_shift / 2); // (r^2 - 0.5px^2)^0.5
				int n_disc_ref = (((n_x + n_FP_factor) * (n_x + n_FP_factor) + (n_y - n_FP_factor / 2) * (n_y - n_FP_factor / 2) - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.25f * n_FP_factor) + (((n_y - n_FP_factor / 2) * (n_y - n_FP_factor / 2) - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.25f * n_FP_factor) + ((n_y * n_y - n_y * n_FP_factor + n_FP_factor * n_FP_factor / 4 - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.25f * n_FP_factor) - n_y + ((n_y * n_y + n_FP_factor * n_FP_factor / 4 - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				int n_disc = int(2.5f * n_FP_factor) - n_y + ((n_y * n_y - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.5f * n_FP_factor) - n_y + ((max(0, n_radius * n_radius - n_FP_factor * n_FP_factor / 4) - n_radius * n_radius) >> n_FP_shift); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.5f * n_FP_factor) - n_y + ((-min((n_radius * n_radius) >> n_FP_shift, n_FP_factor / 4))); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				//int n_disc = int(2.5f * n_FP_factor) - n_y - min((n_radius * n_radius) >> n_FP_shift, n_FP_factor / 4); // discriminant for x+1, y, r, adjusted in y by -1/2 to stay in the middle of the [0, 1] pixel error
				_ASSERTE(abs(n_disc - n_disc_ref) <= 0);
				//int n_disc = (((n_x + n_FP_factor) * (n_x + n_FP_factor) + n_y * n_y - n_radius * n_radius) >> n_FP_shift) - n_FP_factor / 4; // discriminant for x+1, y, r, adjusted by -1/2 to stay in the middle of the [0, 1] pixel error. generates circles with shorter first / last scanline
				//int n_disc = ((1.5) * (1.5) +  - .25  - .25) * n_FP_factor; // note that the above is actually a constant
				//int n_disc = (5 * n_FP_factor - n_radius * 4) / 4 + 2 * n_x + n_FP_factor; // quite magical, but works (the original endpoint + update)
				// adjust for starting with x = .5

				for(bool b_first = true; n_x < n_y;) {
					if(b_first)
						b_first = false;
					else {
						n_x += n_FP_factor;
						if(n_disc < 0) {
							n_disc += 2 * n_x + n_FP_factor;
						} else {
							n_y -= n_FP_factor;
							n_disc += 2 * (n_x - n_y) + n_FP_factor;
						}
					}
					int p_line_coords[12] = {
						-n_x, -n_y, +n_x, /*-n_y,*/ -n_x, +n_y, +n_x, /*+n_y,*/
						-n_y, -n_x, +n_y, /*-n_x,*/ -n_y, +n_x, +n_y/*, +n_x*/
					};
					for(int i = 0; i < 4; ++ i) {
						const int n_y0 = (n_y_center0 + p_line_coords[3 * i + 1]) >> n_FP_shift;
						int n_x0_l = max(0, min(((n_x_center0 + p_line_coords[3 * i + 0]) >> n_FP_shift), n_width)); // no need to take min with n_width - 1, as even if this ends up being n_width, no loop iterations will be taken and no oob access will occur
						const int n_x0_r = max(0, min(((n_x_center0 + p_line_coords[3 * i + 2]) >> n_FP_shift) + 1, n_width));
						const int n_y1 = (n_y_center1 + p_line_coords[3 * i + 1]) >> n_FP_shift;
						int n_x1_l = max(0, min(((n_x_center1 + p_line_coords[3 * i + 0]) >> n_FP_shift), n_width));
						const int n_x1_r = max(0, min(((n_x_center1 + p_line_coords[3 * i + 2]) >> n_FP_shift) + 1, n_width));
						if(n_y0 >= 0 && n_y0 < n_height) {
							for(; n_x0_l < n_x0_r; ++ n_x0_l) {
								if((b_steep && n_y0 + n_x0_l * f_dxdy - f_d0 < 0) ||
								   (!b_steep && n_x0_l + n_y0 * f_dxdy - f_d0 < 0)) // note that the test is not really required if no blending takes place
									p_buffer[n_x0_l + n_width * n_y0] = n_color;//0xffff0000;
							}
						}
						if(n_y1 >= 0 && n_y1 < n_height) {
							for(; n_x1_l < n_x1_r; ++ n_x1_l) {
								if((b_steep && n_y1 + n_x1_l * f_dxdy - f_d1 > 0) ||
								   (!b_steep && n_x1_l + n_y1 * f_dxdy - f_d1 > 0)) // note that the test is not really required if no blending takes place
									p_buffer[n_x1_l + n_width * n_y1] = n_color;//0xffff0000;
							}
						}
					}
				}
			}
			// draw the semi-circle endpoints

			int n_line_end_size = 0;
			if(n_line_end_type == 1 || n_line_end_type == 2)
				n_line_end_size = int(ceil((f_line_width_orig - 1) / 2 * fabs(f_dxdy)));
			// calculate line length extent

			const int n_len = abs(int(floor(f_x1) - floor(f_x0))) + 2 * n_line_end_size;
            CInterpolator lerp(int((f_y0 - n_line_end_size * f_dxdy) * n_FP_factor),
				int((f_y1 + n_line_end_size * f_dxdy) * n_FP_factor), n_len);
			// note this is not adjusted for fractional x

			if(b_steep) {
				/*p_buffer[int(floor(f_y0)) + int(floor(f_x0) * n_width)] = 0xffff0000;
				p_buffer[int(floor(f_y1)) + int(floor(f_x1) * n_width)] = 0xff00ff00;*/
				const int n_y0 = int(floor(f_x0)), n_y1 = int(floor(f_x1));
				const int n_y0b = n_y0 - n_line_end_size, n_y0e = n_y0 + n_line_end_size,
					n_y1b = n_y1 - n_line_end_size + 1, n_y1e = n_y1 + n_line_end_size + 2;
				if(n_y0e < n_y1b) {
					for(int n_y = n_y0b; n_y < n_y0e; ++ n_y, ++ lerp) {
						int n_x_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_x_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_x = n_x_top; n_x <= n_x_bottom; ++ n_x) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height && n_y + n_x * f_dxdy - f_d0 >= 0) // y and x are swapped!
								p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000;
						}
					}
					for(int n_y = n_y0e; n_y < n_y1b; ++ n_y, ++ lerp) {
						int n_x_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_x_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						_ASSERTE(n_x_bottom >= 0 && n_x_top < n_width && n_y >= 0 && n_y < n_height);
						for(int n_x = n_x_top; n_x <= n_x_bottom; ++ n_x) {
							if(n_x >= 0 && n_x < n_width)
								p_buffer[n_x + n_y * n_width] = n_color;
						}
					}
					for(int n_y = n_y1b; n_y < n_y1e; ++ n_y, ++ lerp) {
						int n_x_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_x_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_x = n_x_top; n_x <= n_x_bottom; ++ n_x) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height && n_y + n_x * f_dxdy - f_d1 <= 0) // y and x are swapped!
								p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
						}
					}
				} else {
					for(int n_y = n_y0b; n_y < n_y1e; ++ n_y, ++ lerp) {
						int n_x_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_x_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_x = n_x_top; n_x <= n_x_bottom; ++ n_x) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height &&
							   n_y + n_x * f_dxdy - f_d0 >= 0 && n_y + n_x * f_dxdy - f_d1 <= 0) // y and x are swapped!
								p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
						}
					}
				}
				/*if(p_buffer[int(floor(f_y0)) + int(floor(f_x0) * n_width)] == 0xffff0000)
					printf("b");
				if(p_buffer[int(floor(f_y1)) + int(floor(f_x1) * n_width)] == 0xff00ff00)
					printf("e");*/
			} else {
				/*p_buffer[int(floor(f_x0)) + int(floor(f_y0) * n_width)] = 0xffff0000;
				p_buffer[int(floor(f_x1)) + int(floor(f_y1) * n_width)] = 0xff00ff00;*/
				const int n_x0 = int(floor(f_x0)), n_x1 = int(floor(f_x1));
				const int n_x0b = n_x0 - n_line_end_size, n_x0e = n_x0 + n_line_end_size,
					n_x1b = n_x1 - n_line_end_size + 1, n_x1e = n_x1 + n_line_end_size + 2;
				if(n_x0e < n_x1b) {
					for(int n_x = n_x0b; n_x < n_x0e; ++ n_x, ++ lerp) {
						int n_y_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_y_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_y = n_y_top; n_y <= n_y_bottom; ++ n_y) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d0 >= 0)
								p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000;
						}
					}
					for(int n_x = n_x0e; n_x < n_x1b; ++ n_x, ++ lerp) {
						int n_y_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_y_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						_ASSERTE(n_x >= 0 && n_x < n_width && n_y_bottom >= 0 && n_y_top < n_height);
						for(int n_y = n_y_top; n_y <= n_y_bottom; ++ n_y) {
							if(n_y >= 0 && n_y < n_height)
								p_buffer[n_x + n_y * n_width] = n_color;
						}
					}
					for(int n_x = n_x1b; n_x < n_x1e; ++ n_x, ++ lerp) {
						int n_y_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_y_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_y = n_y_top; n_y <= n_y_bottom; ++ n_y) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d1 <= 0)
								p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
						}
					}
				} else {
					for(int n_x = n_x0b; n_x < n_x1e; ++ n_x, ++ lerp) {
						int n_y_top = (lerp.y() - n_line_extent_top) >> n_FP_shift;
						int n_y_bottom = (lerp.y() + n_line_extent_bottom) >> n_FP_shift;
						for(int n_y = n_y_top; n_y <= n_y_bottom; ++ n_y) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height &&
							   n_x + n_y * f_dxdy - f_d0 >= 0 && n_x + n_y * f_dxdy - f_d1 <= 0)
								p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
						}
					}
				}
				/*if(p_buffer[int(floor(f_x0)) + int(floor(f_y0) * n_width)] == 0xffff0000)
					printf("B");
				if(p_buffer[int(floor(f_x1)) + int(floor(f_y1) * n_width)] == 0xff00ff00)
					printf("E");*/
			}
			// thick lines

			/*if(b_steep) {
				p_buffer[int(floor(f_y0)) + int(floor(f_x0) * n_width)] = 0xffff0000;
				p_buffer[int(floor(f_y1)) + int(floor(f_x1) * n_width)] = 0xff00ff00;
			} else {
				p_buffer[int(floor(f_x0)) + int(floor(f_y0) * n_width)] = 0xffff0000;
				p_buffer[int(floor(f_x1)) + int(floor(f_y1) * n_width)] = 0xff00ff00;
			}*/
			// debug - see where the endpoints are
		}
	}

	/**
	 *	@brief modulates RGBA color by alpha
	 *
	 *	@param[in] n_src is a RGBA color
	 *	@param[in] n_alpha is modulation coefficient (in 0 to 255 range)
	 *
	 *	@return Returns RGBA color, modulated by alpha.
	 */
	static inline uint32_t n_Modulate(uint32_t n_src, int n_alpha)
	{
		_ASSERTE(n_alpha >= 0 && n_alpha <= 0xff);
		return (((((n_src & 0xff00ff) * n_alpha) & 0xff00ff00U) >> 8) |
			   ((((n_src & 0xff00ff00) >> 8) * n_alpha) & 0xff00ff00U));
		// use two ops on pairs of elems
	}

	/**
	 *	@brief modulates RGB color by alpha
	 *
	 *	@param[in] n_src is a RGB color (alpha is ignored)
	 *	@param[in] n_alpha is modulation coefficient (in 0 to 255 range)
	 *
	 *	@return Returns RGB color (alpha is null), modulated by alpha.
	 */
	static inline uint32_t n_Modulate_RGB(uint32_t n_src, int n_alpha)
	{
		_ASSERTE(n_alpha >= 0 && n_alpha <= 0xff);
		return ((((n_src & 0xff00ff) * n_alpha) & 0xff00ff00U) |
			   (((n_src & 0x00ff00) * n_alpha) & 0x00ff0000U)) >> 8; // alpha overflows, and is dammaged
		// use two ops on pairs of elems, save one bit shift
	}

	/**
	 *	@brief modulates R color by alpha
	 *
	 *	@param[in] n_src is a R color (GBA is ignored)
	 *	@param[in] n_alpha is modulation coefficient (in 0 to 255 range)
	 *
	 *	@return Returns R color (the other components are null), modulated by alpha.
	 */
	static inline uint32_t n_Modulate_Red(uint32_t n_src, int n_alpha)
	{
		_ASSERTE(n_alpha >= 0 && n_alpha <= 0xff);
		return (((n_src & 0xff) * n_alpha) & 0xff00) >> 8; // only the red channel is returned
	}

	/**
	 *	@brief modulates greyscale color by alpha
	 *
	 *	@param[in] n_src is a R color (GBA is ignored)
	 *	@param[in] n_alpha is modulation coefficient (in 0 to 255 range)
	 *
	 *	@return Returns greyscale color (RGB is red * alpha, A is alpha), modulated by alpha.
	 */
	static inline uint32_t n_Modulate_Grey(uint8_t n_src, int n_alpha)
	{
		_ASSERTE(n_alpha >= 0 && n_alpha <= 0xff);
		uint32_t n_grey = (uint32_t(n_src) * n_alpha) & 0xff00;
		return (n_alpha << 24) | n_grey | (n_grey << 8) | (n_grey >> 8); // only the red channel is returned
	}

	/**
	 *	@brief blends two RGBA colors based on alpha
	 *
	 *	Calculates r_n_dest = r_n_dest * (255 - n_alpha) + n_src * n_alpha, with RGBA arithmetic.
	 *
	 *	@param[in,out] r_n_dest is destination and left operand (the framebuffer)
	 *	@param[in] n_src is RGBA color (right operand)
	 *	@param[in] n_alpha is modulation coefficient (in 0 to 255 range)
	 */
	static inline void AlphaBlend(uint32_t &r_n_dest, uint32_t n_src, int n_alpha)
	{
		_ASSERTE(n_alpha >= 0 && n_alpha <= 0xff);
		r_n_dest = n_Modulate(r_n_dest, 0xff - n_alpha) + n_Modulate(n_src, n_alpha);
	}

	/**
	 *	@brief draws a solid-color antialiased line with subpixel precision
	 *
	 *	@param[in] f_x0 is a coordinate of the first line point
	 *	@param[in] f_y0 is a coordinate of the first line point
	 *	@param[in] f_x1 is a coordinate of the second line point
	 *	@param[in] f_y1 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] n_line_width is the line width, in pixels
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 *	@note This is somewhat wasteful if the bitmap is grayscale
	 *		since the blending is full RGBA blending.
	 *	@note Thick line rasterization is limited to integer line widths,
	 *		some widths (e.g. 2) do not give nice results.
	 */
	void DrawLine_AA(float f_x0, float f_y0, float f_x1, float f_y1,
		uint32_t n_color, int n_line_width = 1)
	{
		if(n_line_width <= 0)
			return;
		// too thin

		if(!ClipLine(f_x0, f_y0, f_x1, f_y1))
			return;
		// perform simple clipping

		_ASSERTE(max(abs(int(f_x0) - int(f_x1)), abs(int(f_y0) - int(f_y1))) <=
			max(n_width, n_height)); // line lenght is now bound by bitmap size
		bool b_steep = fabs(f_y0 - f_y1) > fabs(f_x0 - f_x1);
		if(b_steep) {
			std::swap(f_x0, f_y0);
			std::swap(f_x1, f_y1);
			// makes sure it is rasterized in the larger dimension
		}
		if(f_x0 > f_x1) {
			std::swap(f_x0, f_x1);
			std::swap(f_y0, f_y1);
		}

		const int n_FP_shift = 16;
		const int n_FP_factor = 65536;
		// added these in order to improve precission on large images

		float f_dxdy = (fabs(f_x1 - f_x0) > 1e-5f)? (f_y1 - f_y0) / (f_x1 - f_x0) : 0;
		int n_gradient = int(n_FP_factor * f_dxdy);
		int n_end_x0 = int(floor(f_x0 + .5f));
		int n_end_x1 = int(floor(f_x1 + .5f));
		// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

		if(n_line_width == 1) {
			if(n_end_x0 == n_end_x1) {
				float f_coverage = f_x1 - f_x0; // length of the line inside pixel
				float f_y_end = f_y0 + f_dxdy * (n_end_x0 - (f_x0 + f_x1) * .5f); // y-position of line center
				// average y in pixel

				int n_y_alpha = int(255 * (f_y_end - floor(f_y_end)));

				if(b_steep) {
					int n_y = n_end_x0, n_x = int(floor(f_y_end));
					if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_y_alpha) * f_coverage));
					if(n_x + 1 >= 0 && n_x + 1 < n_width && n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int(n_y_alpha * f_coverage));
				} else {
					int n_x = n_end_x0, n_y = int(floor(f_y_end));
					if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_y_alpha) * f_coverage));
					if(n_x >= 0 && n_x < n_width && n_y + 1 >= 0 && n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int(n_y_alpha * f_coverage));
				}

				return;
			}
			// in case the line only occupies a single pixel

			float f_end_y0 = f_y0 + f_dxdy * (n_end_x0 - f_x0);
			float f_cov_x0 = ceil(f_x0 + .5f) - (f_x0 + .5f); // how much of line is in the first pixel
			int n_lerp_y = int((f_end_y0 + f_dxdy) * n_FP_factor);
			float f_end_y1 = f_y1 + f_dxdy * (n_end_x1 - f_x1);
			float f_cov_x1 = 1 - (ceil(f_x1 + .5f) - (f_x1 + .5f)); // how much of line is in the last pixel
			//int n_alpha_y0 = 255 - int(255 * (ceil(f_end_y0) - f_end_y0));
			int n_alpha_y0 = int(255 * (f_end_y0 - floor(f_end_y0)));
			int n_alpha_y1 = int(255 * (f_end_y1 - floor(f_end_y1)));
			// calculate aliassing on the end of the lines
			// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

			if(b_steep) {
				if(n_end_x0 >= 0 && n_end_x0 < n_height) {
					int n_y = n_end_x0, n_x = int(floor(f_end_y0));
					if(n_x + 1 >= 0 && n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int((n_alpha_y0) * f_cov_x0));
					if(n_x >= 0 && n_x < n_width)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y0) * f_cov_x0));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_height) {
					int n_y = n_end_x1, n_x = int(floor(f_end_y1));
					if(n_x >= 0 && n_x < n_width)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y1) * f_cov_x1));
					if(n_x + 1 >= 0 && n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int(n_alpha_y1 * f_cov_x1));
				}
				// handle the second endpoint

				for(int n_y = n_end_x0 + 1; n_y < n_end_x1; ++ n_y) {
					int n_x = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, 0xff - n_alpha_0);
					if(n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, n_alpha_0);

					n_lerp_y += n_gradient;
				}
				// draw the line
			} else {
				if(n_end_x0 >= 0 && n_end_x0 < n_width) {
					int n_x = n_end_x0, n_y = int(floor(f_end_y0));
					if(n_y + 1 >= 0 && n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int((n_alpha_y0) * f_cov_x0));
					if(n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y0) * f_cov_x0));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_width) {
					int n_x = n_end_x1, n_y = int(floor(f_end_y1));
					if(n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y1) * f_cov_x1));
					if(n_y + 1 >= 0 && n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int(n_alpha_y1 * f_cov_x1));
				}
				// handle the second endpoint

				for(int n_x = n_end_x0 + 1; n_x < n_end_x1; ++ n_x) {
					int n_y = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, 0xff - n_alpha_0);
					if(n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, n_alpha_0);

					n_lerp_y += n_gradient;
				}
			}
			// thin lines
		} else {
			float f_dx = fabs(f_x1 - f_x0);
			float f_dy = fabs(f_y1 - f_y0);
			float f_thickness_scale = sqrt(f_dx * f_dx + f_dy * f_dy) / max(1.0f, max(f_dx, f_dy));
			n_line_width = max(n_line_width, int(n_line_width * f_thickness_scale + .5f));
			// adjust line thickness based on line angle

			int n_line_extent_top = (n_line_width - 1) / 2;
			int n_line_extent_bottom = (n_line_width - 1) - n_line_extent_top;
			// calculate extent on top and bottom

			float f_end_y0 = f_y0 + f_dxdy * (n_end_x0 - f_x0);
			float f_cov_x0 = ceil(f_x0 + .5f) - (f_x0 + .5f); // how much of line is in the first pixel
			int n_lerp_y = int((f_end_y0 + f_dxdy) * n_FP_factor);
			float f_end_y1 = f_y1 + f_dxdy * (n_end_x1 - f_x1);
			float f_cov_x1 = 1 - (ceil(f_x1 + .5f) - (f_x1 + .5f)); // how much of line is in the last pixel
			//int n_alpha_y0 = 255 - int(255 * (ceil(f_end_y0) - f_end_y0));
			int n_alpha_y0 = int(255 * (f_end_y0 - floor(f_end_y0)));
			int n_alpha_y1 = int(255 * (f_end_y1 - floor(f_end_y1)));
			// calculate aliassing on the end of the lines
			// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

			if(b_steep) {
				if(n_end_x0 >= 0 && n_end_x0 < n_height) {
					int n_y = n_end_x0, n_x = int(floor(f_end_y0));
					if(n_x - n_line_extent_top >= 0 && n_x - n_line_extent_top < n_width)
						AlphaBlend(p_buffer[n_x - n_line_extent_top + n_y * n_width], n_color, int((255 - n_alpha_y0) * f_cov_x0));
					for(int dy = -n_line_extent_top + 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_x + dy >= 0 && n_x + dy < n_width)
							p_buffer[n_x + dy + n_y * n_width] = n_color; //AlphaBlend(p_buffer[n_x + dy + n_y * n_width], n_color, int(255 * f_cov_x0)); // does not give correct results
					}
					if(n_x + n_line_extent_bottom >= 0 && n_x + n_line_extent_bottom < n_width)
						AlphaBlend(p_buffer[n_x + n_line_extent_bottom + n_y * n_width], n_color, int(n_alpha_y0 * f_cov_x0));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_height) {
					int n_y = n_end_x1, n_x = int(floor(f_end_y1));
					if(n_x - n_line_extent_top >= 0 && n_x - n_line_extent_top < n_width)
						AlphaBlend(p_buffer[n_x - n_line_extent_top + n_y * n_width], n_color, int((255 - n_alpha_y1) * f_cov_x1));
					for(int dy = -n_line_extent_top + 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_x + dy >= 0 && n_x + dy < n_width)
							p_buffer[n_x + dy + n_y * n_width] = n_color; //AlphaBlend(p_buffer[n_x + dy + n_y * n_width], n_color, int(255 * f_cov_x1)); // does not give correct results
					}
					if(n_x + n_line_extent_bottom >= 0 && n_x + n_line_extent_bottom < n_width)
						AlphaBlend(p_buffer[n_x + n_line_extent_bottom + n_y * n_width], n_color, int(n_alpha_y1 * f_cov_x1));
				}
				// handle the second endpoint

				for(int n_y = n_end_x0 + 1; n_y < n_end_x1; ++ n_y) {
					int n_x = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					if(n_x >= n_line_extent_top)
						AlphaBlend(p_buffer[n_x - n_line_extent_top + n_y * n_width], n_color, 0xff - n_alpha_0);
					for(int dy = n_line_extent_top - 1; dy > 0; -- dy) {
						if(n_x >= dy)
							p_buffer[n_x - dy + n_y * n_width] = n_color;
					}
					p_buffer[n_x + n_y * n_width] = n_color;
					for(int dy = 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_x + dy < n_width)
							p_buffer[n_x + dy + n_y * n_width] = n_color;
					}
					if(n_x + n_line_extent_bottom < n_width)
						AlphaBlend(p_buffer[n_x + n_line_extent_bottom + n_y * n_width], n_color, n_alpha_0);

					n_lerp_y += n_gradient;
				}
				// draw the line
			} else {
				if(n_end_x0 >= 0 && n_end_x0 < n_width) {
					int n_x = n_end_x0, n_y = int(floor(f_end_y0));
					if(n_y - n_line_extent_top >= 0 && n_y - n_line_extent_top < n_height)
						AlphaBlend(p_buffer[n_x + (n_y - n_line_extent_top) * n_width], n_color, int((255 - n_alpha_y0) * f_cov_x0));
					for(int dy = -n_line_extent_top + 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_y + dy >= 0 && n_y + n_line_extent_bottom < n_height)
							p_buffer[n_x + (n_y + dy) * n_width] = n_color;
					}
					if(n_y + n_line_extent_bottom >= 0 && n_y + n_line_extent_bottom < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + n_line_extent_bottom) * n_width], n_color, int(n_alpha_y0 * f_cov_x0));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_width) {
					int n_x = n_end_x1, n_y = int(floor(f_end_y1));
					if(n_y - n_line_extent_top >= 0 && n_y - n_line_extent_top < n_height)
						AlphaBlend(p_buffer[n_x + (n_y - n_line_extent_top) * n_width], n_color, int((255 - n_alpha_y1) * f_cov_x1));
					for(int dy = -n_line_extent_top + 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_y + dy >= 0 && n_y + n_line_extent_bottom < n_height)
							p_buffer[n_x + (n_y + dy) * n_width] = n_color;
					}
					if(n_y + n_line_extent_bottom >= 0 && n_y + n_line_extent_bottom < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + n_line_extent_bottom) * n_width], n_color, int(n_alpha_y1 * f_cov_x1));
				}
				// handle the second endpoint

				for(int n_x = n_end_x0 + 1; n_x < n_end_x1; ++ n_x) {
					int n_y = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					if(n_y - n_line_extent_top >= 0)
						AlphaBlend(p_buffer[n_x + (n_y - n_line_extent_top) * n_width], n_color, 0xff - n_alpha_0);
					for(int dy = n_line_extent_top - 1; dy > 0; -- dy) {
						if(n_y >= dy)
							p_buffer[n_x + (n_y - dy) * n_width] = n_color;
					}
					p_buffer[n_x + n_y * n_width] = n_color;
					for(int dy = 1; dy < n_line_extent_bottom; ++ dy) {
						if(n_y + dy < n_height)
							p_buffer[n_x + (n_y + dy) * n_width] = n_color;
					}
					if(n_y + n_line_extent_bottom < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + n_line_extent_bottom) * n_width], n_color, n_alpha_0);

					n_lerp_y += n_gradient;
				}
			}
			// thick lines
		}
	}

	/**
	 *	@brief box - circle intersection information
	 */
	class CBoxCircleIsect {
	protected:
		static inline float f_Section(float h, float r = 1)
		{
			return (h < r)? sqrt(r * r - h * h) : 0;
			// http://www.wolframalpha.com/input/?i=r+*+sin%28acos%28x+%2F+r%29%29+%3D+h
		}

		static inline float f_Integral(float x, float h, float r = 1)
		{
			return .5f * (sqrt(1 - x * x / (r * r)) * x * r + r * r * asin(x / r) - 2 * h * x);
			// http://www.wolframalpha.com/input/?i=r+*+sin%28acos%28x+%2F+r%29%29+-+h
		}

		static inline float f_ClampedSection_Area(float x0, float x1, float h, float r)
		{
			_ASSERTE(x0 <= x1);
			//if(x0 > x1)
			//	std::swap(x0, x1); // this must be sorted otherwise we get negative area
			float s = f_Section(h, r);
			return f_Integral(max(-s, min(s, x1)), h, r) -
				f_Integral(max(-s, min(s, x0)), h, r); // integrate the area
		}

	public:
		/**
		 *	@brief calculates area of the intersection of a box and a circle centerd at origin
		 *
		 *	@param[in] x0 is the lower horizontal coordinate of the box
		 *	@param[in] x1 is the upper horizontal coordinate of the box
		 *	@param[in] y0 is the lower vertical coordinate of the box
		 *	@param[in] y1 is the upper vertical coordinate of the box
		 *	@param[in] r is radius of the circle
		 *
		 *	@return Returns the exact area of the intersection of the box and the circle.
		 */
		static inline float f_Area(float x0, float x1, float y0, float y1, float r)
		{
			_ASSERTE(y0 <= y1);
			//if(y0 > y1)
			//	std::swap(y0, y1); // this will simplify the reasoning
			if(y0 < 0) {
				if(y1 < 0) {
					return f_Area(x0, x1, -y1, -y0, r);
					// the box is completely under, just flip it above and try again
				} else {
					return f_Area(x0, x1, 0, -y0, r) + f_Area(x0, x1, 0, y1, r);
					// the box is both above and below, divide it to two boxes and go again
				}
			} else {
				_ASSERTE(y1 >= 0);
				// y0 >= 0, which means that y1 >= 0 also (y1 >= y0) because of the swap at the beginning

				return f_ClampedSection_Area(x0, x1, y0, r) -
					f_ClampedSection_Area(x0, x1, y1, r);
				// area of the lower box minus area of the higher box
			}
		}

		/**
		 *	@brief calculates area of the intersection of a box and a circle
		 *
		 *	@param[in] x0 is the lower horizontal coordinate of the box
		 *	@param[in] x1 is the upper horizontal coordinate of the box
		 *	@param[in] y0 is the lower vertical coordinate of the box
		 *	@param[in] y1 is the upper vertical coordinate of the box
		 *	@param[in] cx is horizontal coordinate of the center of the circle
		 *	@param[in] cy is horizontal vertical of the center of the circle
		 *	@param[in] r is radius of the circle
		 *
		 *	@return Returns the exact area of the intersection of the box and the circle.
		 */
		static float f_Area(float x0, float x1, float y0, float y1, float cx, float cy, float r)
		{
			x0 -= cx; x1 -= cx;
			y0 -= cy; y1 -= cy;
			// get rid of the circle center

			return max(0, f_Area(x0, x1, y0, y1, r)); // this is sometimes slightly imprecise
		}
	};

	/**
	 *	@brief draws antialiased circle
	 *
	 *	@param[in] f_x is horizontal coordinate of the center of the circle
	 *	@param[in] f_y is vertical coordinate of the center of the circle
	 *	@param[in] f_radius is radius of the circle, in pixels
	 *	@param[in] n_color is the line color
	 *	@param[in] f_line_width is line width (default 1, the line is centered on the circle)
	 *
	 *	@note This is using an exact algorithm for computing alpha, which is horribly inefficient.
	 */
	void DrawCircle_AA(float f_x, float f_y, float f_radius, uint32_t n_color, float f_line_width = 1)
	{
		if(f_radius <= 0 || f_line_width <= 0)
			return;

		float f_inner_radius = f_radius - f_line_width / 2;
		float f_outer_radius = f_radius + f_line_width / 2;
		if(f_inner_radius <= 0) { // the line is thicker and fills the inside of the circle completely
			FillCircle_AA(f_x, f_y, f_outer_radius, n_color); // otherwise will divide by zero and have problems
			return;
		}

		const int n_min_x = max(0, min(n_width, int(floor(f_x - f_outer_radius)) - 1)),
			n_min_y = max(0, min(n_height, int(floor(f_y - f_outer_radius)) - 1)),
			n_max_x = max(0, min(n_width, int(ceil(f_x + f_outer_radius)) + 2)),
			n_max_y = max(0, min(n_height, int(ceil(f_y + f_outer_radius)) + 2));
		// ger the raster bounds (leave some space outside for blending)

		const float f_outside = (f_outer_radius + 1.44f) * (f_outer_radius + 1.44f),
			f_inside = (max(f_inner_radius - 1.44f, 0)) * (max(f_inner_radius - 1.44f, 0));
		for(int y = n_min_y; y < n_max_y; ++ y) {
			for(int x = n_min_x; x < n_max_x; ++ x) {
				_ASSERTE(x >= 0 && x < n_width && y >= 0 && y < n_height); // make sure that the clamps work as expected

				float f_cheap = (x - f_x) * (x - f_x) + (y - f_y) * (y - f_y);
				if(f_cheap > f_outside || f_cheap < f_inside)
					continue; // zero alpha

				//float f_dist = CBoxCircleIsect::f_Area(x, x + 1, y, y + 1, f_x, f_y, f_outer_radius) -
				//	CBoxCircleIsect::f_Area(x, x + 1, y, y + 1, f_x, f_y, f_inner_radius);
				float f_dist = CBoxCircleIsect::f_Area(x - f_x, x + 1 - f_x, y - f_y, y + 1 - f_y, f_outer_radius) -
					CBoxCircleIsect::f_Area(x - f_x, x + 1 - f_x, y - f_y, y + 1 - f_y, f_inner_radius);
				// wow, exact integral of area of the circle covering the pixel
				// wow, exact integral of area of the circle covering the pixel

				if(f_dist == 0)
					continue; // zero alpha
				else if(f_dist == 1)
					p_buffer[x + n_width * y] = n_color;
				else
					AlphaBlend(p_buffer[x + n_width * y], n_color, int(255 * f_dist));
			}
		}
		// a simple (expensive) antialiased / subpixel precise circle algorithm

		// t_odo - use an integrator and do it precisely
		// todo - try to adapt bresenham to floats (and verify using the precise code)
	}

	/**
	 *	@brief draws antialiased circle with solid color fill
	 *
	 *	@param[in] f_x is horizontal coordinate of the center of the circle
	 *	@param[in] f_y is vertical coordinate of the center of the circle
	 *	@param[in] f_radius is radius of the circle, in pixels
	 *	@param[in] n_color is the line color
	 *
	 *	@note This is using an exact algorithm for computing alpha, which is horribly inefficient.
	 */
	void FillCircle_AA(float f_x, float f_y, float f_radius, uint32_t n_color)
	{
		if(f_radius <= 0)
			return;
		const int n_min_x = max(0, min(n_width, int(floor(f_x - f_radius)) - 1)),
			n_min_y = max(0, min(n_height, int(floor(f_y - f_radius)) - 1)),
			n_max_x = max(0, min(n_width, int(ceil(f_x + f_radius)) + 2)),
			n_max_y = max(0, min(n_height, int(ceil(f_y + f_radius)) + 2));
		// ger the raster bounds (leave some space outside for blending)

		//const float f_radius2 = f_radius * f_radius;
		const float f_outside = (f_radius + 1.44f) * (f_radius + 1.44f),
			f_inside = (max(f_radius - 1.44f, 0)) * (max(f_radius - 1.44f, 0));
		for(int y = n_min_y; y < n_max_y; ++ y) {
			for(int x = n_min_x; x < n_max_x; ++ x) {
				_ASSERTE(x >= 0 && x < n_width && y >= 0 && y < n_height); // make sure that the clamps work as expected

				/*float f_dist = max(.0f, min(1.0f, .707f * .5f *
					sqrt(f_radius2 - (x - f_x) * (x - f_x) - (y - f_y) * (y - f_y))));*/
				// this is just distance from circle center, that's a cheap approximation

				float f_cheap = (x - f_x) * (x - f_x) + (y - f_y) * (y - f_y);
				//float f_dist = CBoxCircleIsect::f_Area(x, x + 1, y, y + 1, f_x, f_y, f_radius);
				if(f_cheap > f_outside) {
					//_ASSERTE(f_dist < 1.0f / 255);
					continue; // zero alpha
				} else if(f_cheap < f_inside) {
					//_ASSERTE(f_dist > 1 - 1.0f / 255);
					p_buffer[x + n_width * y] = n_color;//0xff00ff00; // full alpha
					continue;
				}

				float f_dist = CBoxCircleIsect::f_Area(x - f_x, x + 1 - f_x, y - f_y, y + 1 - f_y, f_radius);
				// wow, exact integral of area of the circle covering the pixel

				/*if(f_dist == 0)
					continue; // zero alpha
				else if(f_dist == 1)
					p_buffer[x + n_width * y] = n_color;
				else*/
					AlphaBlend(p_buffer[x + n_width * y], n_color, int(255 * f_dist));
			}
		}
		// a simple (expensive) antialiased / subpixel precise circle algorithm

		// t_odo - use an integrator and do it precisely
		// todo - try to adapt bresenham to floats (and verify using the precise code)

#if 0	// this does not work yet
		/*f_x = floor(f_x);
		f_y = floor(f_y);*/
		n_color &= 0xffffff;
		float y = 0, x = f_radius, f_disc = 0, f_r2 = f_radius * f_radius;
		while(y < x) {
			float f_dc = ceil(sqrt(f_r2 - y * y)) - (sqrt(f_r2 - y * y));
			if(f_dc < f_disc)
				x -= 1;
			float x1 = x - 1;
			/*if(int(x1) != int(y))*/ {
				BlendPixel(+x1 + f_x, +y  + f_y, n_color | (int(f_dc * 255) << 24));
				BlendPixel(+y  + f_x, +x1 + f_y, n_color | (int(f_dc * 255) << 24));
			}
			BlendPixel(+x  + f_x, +y  + f_y, n_color | (int(255 - f_dc * 255) << 24));
			BlendPixel(+y  + f_x, +x  + f_y, n_color | (int(255 - f_dc * 255) << 24));
			/*if(int(x) != 0)*/ {
				/*if(int(x1) != int(y))*/ {
					BlendPixel(-x1 + f_x, +y  + f_y, n_color | (int(f_dc * 255) << 24));
					BlendPixel(+y  + f_x, -x1 + f_y, n_color | (int(f_dc * 255) << 24));
				}
				BlendPixel(-x  + f_x, +y  + f_y, n_color | (int(255 - f_dc * 255) << 24));
				BlendPixel(+y  + f_x, -x  + f_y, n_color | (int(255 - f_dc * 255) << 24));
			}
			/*if(int(y) != 0)*/ {
				/*if(int(x1) != int(y))*/ {
					BlendPixel(-y  + f_x, +x1 + f_y, n_color | (int(f_dc * 255) << 24));
					BlendPixel(-y  + f_x, -x1 + f_y, n_color | (int(f_dc * 255) << 24));
					BlendPixel(+x1 + f_x, -y  + f_y, n_color | (int(f_dc * 255) << 24));
					BlendPixel(-x1 + f_x, -y  + f_y, n_color | (int(f_dc * 255) << 24));
				}
				BlendPixel(-y  + f_x, +x  + f_y, n_color | (int(255 - f_dc * 255) << 24));
				BlendPixel(-y  + f_x, -x  + f_y, n_color | (int(255 - f_dc * 255) << 24));
				BlendPixel(+x  + f_x, -y  + f_y, n_color | (int(255 - f_dc * 255) << 24));
				BlendPixel(-x  + f_x, -y  + f_y, n_color | (int(255 - f_dc * 255) << 24));
			}
			y += 1;
			f_disc = f_dc;
		}
#endif // 0
	}

	/**
	 *	@brief draws a solid-color antialiased line with subpixel precision
	 *
	 *	@param[in] f_x0 is a coordinate of the first line point
	 *	@param[in] f_y0 is a coordinate of the first line point
	 *	@param[in] f_x1 is a coordinate of the second line point
	 *	@param[in] f_y1 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] f_line_width is the line width, in pixels
	 *	@param[in] n_line_end_type is line end type (0 = natural / fastest, 1 = bevel, 2 = round)
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 *	@note This is somewhat wasteful if the bitmap is grayscale
	 *		since the blending is full RGBA blending.
	 *	@note Only the RGB parts of the color are used. The alpha is assumed to be 0xff.
	 *
	 *	@todo To draw a path with alpha less than 100%, the rasterizer actually needs to know
	 *		about all the lines, otherwise the overlapping parts will get more alpha.
	 */
	void DrawLine_AA2(float f_x0, float f_y0, float f_x1, float f_y1,
		uint32_t n_color, float f_line_width = 1, int n_line_end_type = 2)
	{
		_ASSERTE(n_line_end_type >= 0 && n_line_end_type <= 2);
		if(f_line_width <= 0)
			return;
		// too thin

		if(!ClipLine(f_x0, f_y0, f_x1, f_y1))
			return;
		// perform simple clipping

		_ASSERTE(max(abs(int(f_x0) - int(f_x1)), abs(int(f_y0) - int(f_y1))) <=
			max(n_width, n_height)); // line lenght is now bound by bitmap size
		const bool b_steep = fabs(f_y0 - f_y1) > fabs(f_x0 - f_x1);
		if(b_steep) {
			std::swap(f_x0, f_y0);
			std::swap(f_x1, f_y1);
			// makes sure it is rasterized in the larger dimension
		}
		if(f_x0 > f_x1) {
			std::swap(f_x0, f_x1);
			std::swap(f_y0, f_y1);
		}

		const int n_FP_shift = 16;
		const int n_FP_factor = 65536;
		// added these in order to improve precission on large images

		const float f_dxdy = (fabs(f_x1 - f_x0) > 1e-5f)? (f_y1 - f_y0) / (f_x1 - f_x0) : 0;
		const int n_gradient = int(n_FP_factor * f_dxdy);
		int n_end_x0 = int(floor(f_x0 + .5f));
		int n_end_x1 = int(floor(f_x1 + .5f));
		// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

		const float f_line_width_orig = f_line_width;
		f_line_width = f_line_width * sqrt(1 + f_dxdy * f_dxdy);
		// update the line width, based on the slope of the line
		// note that this is corrected for b_steep

		const float f_cov_x0 = sqrt(1 - (f_x0 + .5f - floor(f_x0 + .5f)));//ceil(f_x0 + .5f) - (f_x0 + .5f); // how much of line is in the first pixel
		const float f_cov_x1 = sqrt(f_x1 + .5f - floor(f_x1 + .5f));//1 - (ceil(f_x1 + .5f) - (f_x1 + .5f)); // how much of line is in the last pixel
		// note that we take a sqrt here to fill in the edges slightly more to get rid of seams where two
		// lines meet (even though they overlap exactly, blend(blend(white, black, 50%), black, 50%) != black)

		const float f_end_y0 = f_y0 + f_dxdy * (n_end_x0 - f_x0);
		int n_lerp_y = int((f_end_y0 + f_dxdy) * n_FP_factor);
		const float f_end_y1 = f_y1 + f_dxdy * (n_end_x1 - f_x1);
		// the same for both branches

		if(f_line_width <= 1) {
			_ASSERTE(fabs(f_x0 - f_x1) >= fabs(f_y0 - f_y1)); // make sure we're not missing some cases
			if(n_end_x0 == n_end_x1) {
				float f_coverage = (f_x1 - f_x0) * f_line_width; // length of the line inside a pixel
				float f_y_end = (f_y0 + f_y1) * .5f; // average y in pixel //f_y0 + f_dxdy * ((f_x0 + f_x1) * .5f - n_end_x0); // y-position of line center
				int n_y_alpha = int(255 * (f_y_end - floor(f_y_end)));

				int n_x = n_end_x0, n_y = int(floor(f_y_end));
				if(b_steep) {
					std::swap(n_x, n_y);
					if(n_x + 1 >= 0 && n_x + 1 < n_width && n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int(n_y_alpha * f_coverage));
				} else if(n_x >= 0 && n_x < n_width && n_y + 1 >= 0 && n_y + 1 < n_height)
					AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int(n_y_alpha * f_coverage));
				if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
					AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_y_alpha) * f_coverage));

				return;
			}
			// in case the line only occupies a single pixel

			const int n_alpha_y0 = int(255 * (f_end_y0 - floor(f_end_y0)));
			const int n_alpha_y1 = int(255 * (f_end_y1 - floor(f_end_y1)));
			// calculate aliassing on the end of the lines
			// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

			if(b_steep) {
				if(n_end_x0 >= 0 && n_end_x0 < n_height) {
					int n_y = n_end_x0, n_x = int(floor(f_end_y0));
					if(n_x + 1 >= 0 && n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int((n_alpha_y0) * (f_cov_x0 * f_line_width)));
					if(n_x >= 0 && n_x < n_width)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y0) * (f_cov_x0 * f_line_width)));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_height) {
					int n_y = n_end_x1, n_x = int(floor(f_end_y1));
					if(n_x >= 0 && n_x < n_width)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y1) * (f_cov_x1 * f_line_width)));
					if(n_x + 1 >= 0 && n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int(n_alpha_y1 * (f_cov_x1 * f_line_width)));
				}
				// handle the second endpoint

				for(int n_y = n_end_x0 + 1; n_y < n_end_x1; ++ n_y, n_lerp_y += n_gradient) {
					int n_x = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((0xff - n_alpha_0) * f_line_width));
					if(n_x + 1 < n_width)
						AlphaBlend(p_buffer[n_x + 1 + n_y * n_width], n_color, int(n_alpha_0 * f_line_width));
				}
				// draw the line
			} else {
				if(n_end_x0 >= 0 && n_end_x0 < n_width) {
					int n_x = n_end_x0, n_y = int(floor(f_end_y0));
					if(n_y + 1 >= 0 && n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int((n_alpha_y0) * (f_cov_x0 * f_line_width)));
					if(n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y0) * (f_cov_x0 * f_line_width)));
				}
				// handle the first endpoint

				if(n_end_x1 >= 0 && n_end_x1 < n_width) {
					int n_x = n_end_x1, n_y = int(floor(f_end_y1));
					if(n_y >= 0 && n_y < n_height)
						AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((255 - n_alpha_y1) * (f_cov_x1 * f_line_width)));
					if(n_y + 1 >= 0 && n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int(n_alpha_y1 * (f_cov_x1 * f_line_width)));
				}
				// handle the second endpoint

				for(int n_x = n_end_x0 + 1; n_x < n_end_x1; ++ n_x, n_lerp_y += n_gradient) {
					int n_y = n_lerp_y >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height);
					int n_alpha_0 = (n_lerp_y >> (n_FP_shift - 8)) & 0xff;
					AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int((0xff - n_alpha_0) * f_line_width));
					if(n_y + 1 < n_height)
						AlphaBlend(p_buffer[n_x + (n_y + 1) * n_width], n_color, int(n_alpha_0 * f_line_width));
				}
			}
			// thin lines
		} else {
			const float f_line_extent_top = (f_line_width - 1) * .5f;
			const float f_line_extent_bottom = f_line_extent_top; // equal in floating point
			const int n_line_extent_top = int(f_line_extent_top * n_FP_factor);
			const int n_line_extent_bottom = n_line_extent_top; // just semantics, negligible error //int(f_line_width * n_FP_factor - n_line_extent_top); // not entirely equal in fixed point
			// calculate extent on top and bottom

			if(!n_line_end_type) {
				if(n_end_x0 == n_end_x1) {
					float f_coverage = f_x1 - f_x0; // length of the line inside a pixel
					float f_y_end = f_y0 + f_dxdy * (n_end_x0 - (f_x0 + f_x1) * .5f); // y-position of line center
					// average y in pixel

					int n_y_top = int(floor(f_y_end - f_line_extent_top));
					int n_y_bottom = int(floor(f_y_end + f_line_extent_bottom));
					int n_y_alpha_top = 255 - int(255 * (f_y_end - f_line_extent_top - floor(f_y_end - f_line_extent_top)));
					int n_y_alpha_bottom = int(255 * (f_y_end + f_line_extent_bottom - floor(f_y_end + f_line_extent_bottom)));

					if(b_steep) {
						int n_y = n_end_x0, n_x = n_y_top;
						if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
							AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(n_y_alpha_top * f_coverage));
						for(++ n_x; n_x <= n_y_bottom; ++ n_x) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
								AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_coverage));
						}
						_ASSERTE(n_x == n_y_bottom + 1);
						if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
							AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(n_y_alpha_bottom * f_coverage));
					} else {
						int n_x = n_end_x0, n_y = n_y_top;
						if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
							AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(n_y_alpha_top * f_coverage));
						for(++ n_y; n_y <= n_y_bottom; ++ n_y) {
							if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
								AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_coverage));
						}
						_ASSERTE(n_y == n_y_bottom + 1);
						if(n_x >= 0 && n_x < n_width && n_y >= 0 && n_y < n_height)
							AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(n_y_alpha_bottom * f_coverage));
					}

					return;
				}
				// in case the line only occupies a single pixel

				{
					const int n_alpha_y0_top = 255 - int(255 * (f_end_y0 - f_line_extent_top - floor(f_end_y0 - f_line_extent_top)));
					const int n_alpha_y1_top = 255 - int(255 * (f_end_y1 - f_line_extent_top - floor(f_end_y1 - f_line_extent_top)));
					const int n_alpha_y0_bottom = int(255 * (f_end_y0 + f_line_extent_bottom - floor(f_end_y0 + f_line_extent_bottom)));
					const int n_alpha_y1_bottom = int(255 * (f_end_y1 + f_line_extent_bottom - floor(f_end_y1 + f_line_extent_bottom)));
					// calculate aliassing on the end of the lines
					// note the .5 are important otherwise antialiassing discontinuities occur in the first quadrant

					if(b_steep) {
						if(n_end_x0 >= 0 && n_end_x0 < n_height) {
							int n_y = n_end_x0, n_x_top = int(floor(f_end_y0 - f_line_extent_top)),
								n_x_bottom = int(floor(f_end_y0 + f_line_extent_bottom));
							if(n_x_top >= 0 && n_x_top < n_width)
								AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, int(n_alpha_y0_top * f_cov_x0));
							for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
								if(n_x >= 0 && n_x < n_width) {
									//p_buffer[n_x + n_y * n_width] = n_color;
									AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_cov_x0)); // seems ok now
								}
							}
							if(++ n_x_bottom >= 0 && n_x_bottom < n_width)
								AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, int(n_alpha_y0_bottom * f_cov_x0));
						}
						// handle the first endpoint

						if(n_end_x1 >= 0 && n_end_x1 < n_height) {
							int n_y = n_end_x1, n_x_top = int(floor(f_end_y1 - f_line_extent_top)),
								n_x_bottom = int(floor(f_end_y1 + f_line_extent_bottom));
							if(n_x_top >= 0 && n_x_top < n_width)
								AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, int(n_alpha_y1_top * f_cov_x1));
							for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
								if(n_x >= 0 && n_x < n_width) {
									//p_buffer[n_x + n_y * n_width] = n_color;
									AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_cov_x1)); // seems ok now
								}
							}
							if(++ n_x_bottom >= 0 && n_x_bottom < n_width)
								AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, int(n_alpha_y1_bottom * f_cov_x1));
						}
						// handle the second endpoint
					} else {
						if(n_end_x0 >= 0 && n_end_x0 < n_width) {
							int n_x = n_end_x0, n_y_top = int(floor(f_end_y0 - f_line_extent_top)),
								n_y_bottom = int(floor(f_end_y0 + f_line_extent_bottom));
							if(n_y_top >= 0 && n_y_top < n_height)
								AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, int(n_alpha_y0_top * f_cov_x0));
							for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
								if(n_y >= 0 && n_y < n_height) {
									//p_buffer[n_x + n_y * n_width] = n_color;
									AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_cov_x0)); // seems ok now
								}
							}
							if(++ n_y_bottom >= 0 && n_y_bottom < n_height)
								AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, int(n_alpha_y0_bottom * f_cov_x0));
						}
						// handle the first endpoint

						if(n_end_x1 >= 0 && n_end_x1 < n_width) {
							int n_x = n_end_x1, n_y_top = int(floor(f_end_y1 - f_line_extent_top)),
								n_y_bottom = int(floor(f_end_y1 + f_line_extent_bottom));
							if(n_y_top >= 0 && n_y_top < n_height)
								AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, int(n_alpha_y1_top * f_cov_x1));
							for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
								if(n_y >= 0 && n_y < n_height) {
									//p_buffer[n_x + n_y * n_width] = n_color;
									AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * f_cov_x1)); // seems ok now
								}
							}
							if(++ n_y_bottom >= 0 && n_y_bottom < n_height)
								AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, int(n_alpha_y1_bottom * f_cov_x1));
						}
						// handle the second endpoint
					}
				}
				// thick endpoints, type 0

				++ n_end_x0;
				-- n_end_x1;
				// clip the line so that its ends are not over-drawn
			} else if(n_line_end_type == 1 || n_line_end_type == 2) {
				int n_line_end_size = int(ceil((f_line_width_orig - 1) / 2 * fabs(f_dxdy))) + 1; // t_odo - carry the changes to SP as well // does not seem to make a difference
				// calculate line length extent

				const float k = (n_line_end_type == 1)? .75f : 0; // if it ends with discs, the discs take care of the seams. if it ends with bevels, we need to extend the line a bit in order to not have seams. natural ends will have seams.
				const float f_d0 = f_x0 - k + f_y0 * f_dxdy;
				const float f_d1 = f_x1 + k + f_y1 * f_dxdy; // t_odo - debug these, there must be no overdraw between the adjacent lines

				const int n_x_size = (b_steep)? n_height : n_width;
				_ASSERTE(n_end_x0 >= 0 && n_end_x0 <= n_x_size);
				_ASSERTE(n_end_x1 >= 0 && n_end_x1 <= n_x_size);
				const int n_x0b = max(n_end_x0 - n_line_end_size, 0),
					n_x0e = min(n_end_x0 + n_line_end_size, n_x_size),
					n_x1b = min(n_end_x1 - n_line_end_size + 1, n_x_size),
					n_x1e = min(n_end_x1 + n_line_end_size + 2, n_x_size);
				int n_lerp_y0 = n_lerp_y + n_gradient * (-n_line_end_size - 1 + max(0, n_line_end_size - n_end_x0)); // starts at n_x0b
				int n_lerp_y1 = n_lerp_y0 + n_gradient * (n_x1b - n_x0b);
				_ASSERTE(n_x0b >= 0 && n_x0b < n_x_size);
				_ASSERTE(n_x0e >= n_x1b || n_x0e >= 0 && n_x0e <= n_x_size); // no need to use n_x_size - 1, it is an exclusive limit
				_ASSERTE(n_x0e >= n_x1b || n_x1b >= 0 && n_x1b < n_x_size);
				_ASSERTE(n_x1e >= 0 && n_x1e <= n_x_size); // no need to use n_x_size - 1, it is an exclusive limit

				if(n_line_end_type == 1) {
					if(b_steep) {
						if(n_x0e < n_x1b) {
							for(int n_y = n_x0b; n_y < n_x0e; ++ n_y, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, int(n_alpha_top * max(0, min(1, n_y + n_x_top * f_dxdy - f_d0))));
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width && n_y + n_x * f_dxdy - f_d0 >= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, n_y + n_x * f_dxdy - f_d0))));
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, int(n_alpha_bottom * max(0, min(1, n_y + n_x_bottom * f_dxdy - f_d0))));
							}
							for(int n_y = n_x1b; n_y < n_x1e; ++ n_y, n_lerp_y1 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y1 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y1 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y1 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y1 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, int(n_alpha_top * max(0, min(1, f_d1 - n_y - n_x_top * f_dxdy))));
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width && n_y + n_x * f_dxdy - f_d1 <= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, f_d1 - n_y - n_x * f_dxdy))));
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, int(n_alpha_bottom * max(0, min(1, f_d1 - n_y - n_x_bottom * f_dxdy))));
							}
						} else {
							for(int n_y = n_x0b; n_y < n_x1e; ++ n_y, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d0 >= 0 && n_y + n_x_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, int(n_alpha_top * max(0, min(1, n_y + n_x_top * f_dxdy - f_d0)) * max(0, min(1, f_d1 - n_y - n_x_top * f_dxdy))));
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width &&
									   n_y + n_x * f_dxdy - f_d0 >= 0 && n_y + n_x * f_dxdy - f_d1 <= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, n_y + n_x * f_dxdy - f_d0)) * max(0, min(1, f_d1 - n_y - n_x * f_dxdy))));
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d0 >= 0 && n_y + n_x_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, int(n_alpha_bottom * max(0, min(1, n_y + n_x_bottom * f_dxdy - f_d0)) * max(0, min(1, f_d1 - n_y - n_x_bottom * f_dxdy))));
							}
						}
					} else {
						if(n_x0e < n_x1b) {
							for(int n_x = n_x0b; n_x < n_x0e; ++ n_x, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, int(n_alpha_top * max(0, min(1, n_y_top * f_dxdy + n_x - f_d0))));
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d0 >= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000;
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, n_y * f_dxdy + n_x - f_d0))));
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, int(n_alpha_bottom * max(0, min(1, n_y_bottom * f_dxdy + n_x - f_d0))));
							}
							for(int n_x = n_x1b; n_x < n_x1e; ++ n_x, n_lerp_y1 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y1 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y1 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y1 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y1 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, int(n_alpha_top * max(0, min(1, f_d1 - n_y_top * f_dxdy - n_x))));
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d1 <= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, f_d1 - n_y * f_dxdy - n_x))));
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, int(n_alpha_bottom * max(0, min(1, f_d1 - n_y_bottom * f_dxdy - n_x))));
							}
						} else {
							for(int n_x = n_x0b; n_x < n_x1e; ++ n_x, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d0 >= 0 && n_x + n_y_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, int(n_alpha_top * max(0, min(1, n_y_top * f_dxdy + n_x - f_d0)) * max(0, min(1, f_d1 - n_y_top * f_dxdy - n_x))));
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height &&
									   n_x + n_y * f_dxdy - f_d0 >= 0 && n_x + n_y * f_dxdy - f_d1 <= 0)
										//p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
										AlphaBlend(p_buffer[n_x + n_y * n_width], n_color, int(255 * max(0, min(1, n_y * f_dxdy + n_x - f_d0)) * max(0, min(1, f_d1 - n_y * f_dxdy - n_x))));
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d0 >= 0 && n_x + n_y_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, int(n_alpha_bottom * max(0, min(1, n_y_bottom * f_dxdy + n_x - f_d0)) * max(0, min(1, f_d1 - n_y_bottom * f_dxdy - n_x))));
							}
						}
					}
					// thick endpoints, type 1
				} else {
					if(b_steep) {
						if(n_x0e < n_x1b) {
							for(int n_y = n_x0b; n_y < n_x0e; ++ n_y, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, n_alpha_top);
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width && n_y + n_x * f_dxdy - f_d0 >= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, n_alpha_bottom);
							}
							for(int n_y = n_x1b; n_y < n_x1e; ++ n_y, n_lerp_y1 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y1 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y1 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y1 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y1 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d1 <= 0) // t_odo - use <= / >= everywhere
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, n_alpha_top);
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width && n_y + n_x * f_dxdy - f_d1 <= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, n_alpha_bottom);
							}
						} else {
							for(int n_y = n_x0b; n_y < n_x1e; ++ n_y, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_y >= 0 && n_y < n_height); // no guarantees about x
								int n_x_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_x_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_x_top >= 0 && n_x_top < n_width && n_y + n_x_top * f_dxdy - f_d0 >= 0 && n_y + n_x_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, n_alpha_top);
								for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
									if(n_x >= 0 && n_x < n_width &&
									   n_y + n_x * f_dxdy - f_d0 >= 0 && n_y + n_x * f_dxdy - f_d1 <= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
								}
								if(++ n_x_bottom >= 0 && n_x_bottom < n_width && n_y + n_x_bottom * f_dxdy - f_d0 >= 0 && n_y + n_x_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, n_alpha_bottom);
							}
						}
					} else {
						if(n_x0e < n_x1b) {
							for(int n_x = n_x0b; n_x < n_x0e; ++ n_x, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, n_alpha_top);
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d0 >= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xffff0000;
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d0 >= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, n_alpha_bottom);
							}
							for(int n_x = n_x1b; n_x < n_x1e; ++ n_x, n_lerp_y1 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y1 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y1 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y1 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y1 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, n_alpha_top);
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height && n_x + n_y * f_dxdy - f_d1 <= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xff0000ff;
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, n_alpha_bottom);
							}
						} else {
							for(int n_x = n_x0b; n_x < n_x1e; ++ n_x, n_lerp_y0 += n_gradient) {
								_ASSERTE(n_x >= 0 && n_x < n_width); // no guarantees about y
								int n_y_top = (n_lerp_y0 - n_line_extent_top) >> n_FP_shift;
								int n_y_bottom = (n_lerp_y0 + n_line_extent_bottom) >> n_FP_shift;
								int n_alpha_top = 255 - (((n_lerp_y0 - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
								int n_alpha_bottom = ((n_lerp_y0 + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
								if(n_y_top >= 0 && n_y_top < n_height && n_x + n_y_top * f_dxdy - f_d0 >= 0 && n_x + n_y_top * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, n_alpha_top);
								for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
									if(n_y >= 0 && n_y < n_height &&
									   n_x + n_y * f_dxdy - f_d0 >= 0 && n_x + n_y * f_dxdy - f_d1 <= 0)
										p_buffer[n_x + n_y * n_width] = n_color;//0xff00ff00;
								}
								if(++ n_y_bottom >= 0 && n_y_bottom < n_height && n_x + n_y_bottom * f_dxdy - f_d0 >= 0 && n_x + n_y_bottom * f_dxdy - f_d1 <= 0)
									AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, n_alpha_bottom);
							}
						}
					}
					// thick endpoints, type 1, no antialiassing of the ends

					const float p_x[] = {f_x0 + .5f, f_x1 + .5f}, p_y[] = {f_y0 + .5f, f_y1 + .5f}, p_d[] = {f_d0, f_d1};
					for(int n_end = 0; n_end < 2; ++ n_end) {
						const float f_x = ((b_steep)? p_y : p_x)[n_end],
							f_y = ((b_steep)? p_x : p_y)[n_end],
							f_d = p_d[n_end], f_radius = f_line_width_orig / 2;
						const int n_sign = (n_end)? 1 : -1;
						const int n_min_x = max(0, min(n_width, int(floor(f_x - f_radius)) - 1)),
							n_min_y = max(0, min(n_height, int(floor(f_y - f_radius)) - 1)),
							n_max_x = max(0, min(n_width, int(ceil(f_x + f_radius)) + 2)),
							n_max_y = max(0, min(n_height, int(ceil(f_y + f_radius)) + 2));
						// ger the raster bounds (leave some space outside for blending)

						const float f_outside = (f_radius + 1.44f) * (f_radius + 1.44f),
							f_inside = (max(f_radius - 1.44f, 0)) * (max(f_radius - 1.44f, 0));
						for(int y = n_min_y; y < n_max_y; ++ y) {
							for(int x = n_min_x; x < n_max_x; ++ x) {
								_ASSERTE(x >= 0 && x < n_width && y >= 0 && y < n_height); // make sure that the clamps work as expected

								if(((b_steep)? y + x * f_dxdy - f_d : x + y * f_dxdy - f_d) * n_sign < 0)
									continue;

								float f_cheap = (x - f_x) * (x - f_x) + (y - f_y) * (y - f_y);
								//float f_dist = CBoxCircleIsect::f_Area(x, x + 1, y, y + 1, f_x, f_y, f_radius);
								if(f_cheap > f_outside) {
									//_ASSERTE(f_dist < 1.0f / 255);
									continue; // zero alpha
								} else if(f_cheap < f_inside) {
									//_ASSERTE(f_dist > 1 - 1.0f / 255);
									p_buffer[x + n_width * y] = n_color;//0xff00ff00; // full alpha
									continue;
								}

								float f_dist = CBoxCircleIsect::f_Area(x - f_x,
									x + 1 - f_x, y - f_y, y + 1 - f_y, f_radius);
								// wow, exact integral of area of the circle covering the pixel

								AlphaBlend(p_buffer[x + n_width * y], n_color, int(255 * f_dist));
							}
						}
						// a simple (expensive) antialiased / subpixel precise circle algorithm
					}
					// t_odo - draw antialiased discs at either end
					// todo - make this more efficient, this is outrageously expensive
				}

				n_lerp_y += n_gradient * (n_line_end_size - 1); // note that n_lerp_y has a single dxdy offset in it already (always used by the thin line rasterizer)
				n_end_x0 += n_line_end_size;
				n_end_x1 -= n_line_end_size;
				// clip the line so that its ends are not over-drawn
			}

			if(b_steep) {
				for(int n_y = n_end_x0; n_y <= n_end_x1; ++ n_y, n_lerp_y += n_gradient) {
					int n_x_top = (n_lerp_y - n_line_extent_top) >> n_FP_shift;
					int n_x_bottom = (n_lerp_y + n_line_extent_bottom) >> n_FP_shift;
					_ASSERTE(n_x_bottom >= 0 && n_x_top < n_width && n_y >= 0 && n_y < n_height); // only partially bound
					int n_alpha_top = 255 - (((n_lerp_y - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
					int n_alpha_bottom = ((n_lerp_y + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
					if(n_x_top >= 0)
						AlphaBlend(p_buffer[n_x_top + n_y * n_width], n_color, n_alpha_top);
					for(int n_x = n_x_top + 1; n_x <= n_x_bottom; ++ n_x) {
						if(n_x >= 0 && n_x < n_width)
							p_buffer[n_x + n_y * n_width] = n_color; // full coverage
					}
					if(++ n_x_bottom < n_width)
						AlphaBlend(p_buffer[n_x_bottom + n_y * n_width], n_color, n_alpha_bottom);
				}
				// draw the line
			} else {
				for(int n_x = n_end_x0; n_x <= n_end_x1; ++ n_x, n_lerp_y += n_gradient) {
					int n_y_top = (n_lerp_y - n_line_extent_top) >> n_FP_shift;
					int n_y_bottom = (n_lerp_y + n_line_extent_bottom) >> n_FP_shift;
					_ASSERTE(n_x >= 0 && n_x < n_width && n_y_bottom >= 0 && n_y_top < n_height); // only partially bound
					int n_alpha_top = 255 - (((n_lerp_y - n_line_extent_top) >> (n_FP_shift - 8)) & 0xff);
					int n_alpha_bottom = ((n_lerp_y + n_line_extent_bottom) >> (n_FP_shift - 8)) & 0xff;
					if(n_y_top >= 0)
						AlphaBlend(p_buffer[n_x + n_y_top * n_width], n_color, n_alpha_top);
					for(int n_y = n_y_top + 1; n_y <= n_y_bottom; ++ n_y) {
						if(n_y >= 0 && n_y < n_height)
							p_buffer[n_x + n_y * n_width] = n_color; // full coverage
					}
					if(++ n_y_bottom < n_height)
						AlphaBlend(p_buffer[n_x + n_y_bottom * n_width], n_color, n_alpha_bottom);
				}
			}
			// thick lines
		}
	}

	/**
	 *	@brief draws a solid-color line
	 *
	 *	@param[in] p_z_buffer is pointer to the buffer containing 1/z values
	 *		(a block of memory with the same dimensions and addressing as this bitmap)
	 *	@param[in] x1 is a coordinate of the first line point
	 *	@param[in] y1 is a coordinate of the first line point
	 *	@param[in] z1 is a coordinate of the first line point
	 *	@param[in] x2 is a coordinate of the second line point
	 *	@param[in] y2 is a coordinate of the second line point
	 *	@param[in] z2 is a coordinate of the second line point
	 *	@param[in] n_color is the line color
	 *	@param[in] n_line_width is the line width, in pixels
	 *
	 *	@note This performs array boundary checking,
	 *		coordinates outside the bitmap are ok.
	 *	@note Depth test behavior is equivalent to that of GL_LESS.
	 *
	 *	@todo Write templated variant of this function to support interpolation of more
	 *		coordinates and custom depth test / shading.
	 *	@todo Implement a better clipping (large offscreen lines
	 *		are rasterized rather slow now).
	 */
	void DrawLine_ZBuffer(float *p_z_buffer, float x1, float y1, float z1,
		float x2, float y2, float z2, uint32_t n_color, int n_line_width = 1)
	{
		float x = floor(x1);
		float y = floor(y1);
		int xlen = int(x1) - int(x2);
		int ylen = int(y1) - int(y2);
		int len = abs(((abs(ylen) > abs(xlen))? ylen : xlen));
		float stepx = float(xlen) / len;
		float stepy = float(ylen) / len;
		float z = 1 / z1;
		float zstep = (1 / z2 - 1 / z1) / len;

		if(n_line_width == 1) {
			for(int i = 0; i <= (int)len; ++ i) {
				if(x >= 0 && x < n_width && y >= 0 && y < n_height && p_z_buffer[int(x) + n_width * int(y)] > z) {
					p_z_buffer[int(x) + n_width * int(y)] = z;
					p_buffer[int(x) + n_width * int(y)] = n_color;
				}
				x -= stepx;
				y -= stepy;
				z += zstep;
			}
		} else {
			int n_start = -n_line_width / 2;
			int n_end = n_start + n_line_width;

			if(abs(xlen) < abs(ylen)) {
				for(int i = 0; i <= (int)len; ++ i) {
					if(y >= 0 && y < n_height) {
						for(int xs = max(0, int(x + n_start)), xe = min(n_width, int(x + n_end)); xs < xe; ++ xs) {
							if(p_z_buffer[xs + n_width * int(y)] > z) {
								p_z_buffer[xs + n_width * int(y)] = z;
								p_buffer[xs + n_width * int(y)] = n_color;
							}
						}
					}
					x -= stepx;
					y -= stepy;
					z += zstep;
				}
			} else {
				for(int i = 0; i <= (int)len; ++ i) {
					if(x >= 0 && x < n_width) {
						for(int ys = max(0, int(y + n_start)), ye = min(n_height, int(y + n_end)); ys < ye; ++ ys) {
							if(p_z_buffer[int(x) + n_width * ys] > z) {
								p_z_buffer[int(x) + n_width * ys] = z;
								p_buffer[int(x) + n_width * ys] = n_color;
							}
						}
					}
					x -= stepx;
					y -= stepy;
					z += zstep;
				}
			}
		}
		// quick and dirty
	}

	/**
	 *	@brief draws a solid-color triangle
	 *
	 *	@param[in] p_z_buffer is pointer to the buffer containing 1/z values
	 *		(a block of memory with the same dimensions and addressing as this bitmap)
	 *	@param[in] p_vertex is a list of the triangle vertices
	 *	@param[in] n_color is the fill color
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 *	@note Depth test behavior is equivalent to that of GL_LESS.
	 *
	 *	@todo Write templated variant of this function to support interpolation of more
	 *		coordinates and custom depth test / shading.
	 */
	void DrawTriangle_ZBuffer(float *p_z_buffer, const float p_vertex[3][3], uint32_t n_color)
	{
		int n_min_y = int(floor(min(p_vertex[0][1], min(p_vertex[1][1], p_vertex[2][1]))));
		int n_max_y = int(floor(max(p_vertex[0][1], max(p_vertex[1][1], p_vertex[2][1]))));
		// find the top / bottom y

		if(n_min_y < 0)
			n_min_y = 0;
		if(n_max_y >= n_height)
			n_max_y = n_height - 1;
		// make sure we don't compute pixels that are not displayed in the end

		p_z_buffer += n_width * n_min_y;
		uint32_t *p_color_buffer = p_buffer + n_width * n_min_y;
		for(int y = n_min_y; y <= n_max_y; ++ y, p_z_buffer += n_width, p_color_buffer += n_width) {
			int n_point_num = 0;
			float p_segment[2][3];
			for(int i = 0, j = 2; i < 3; j = i ++) {
				const float *p_a = p_vertex[i];
				const float *p_b = p_vertex[j];

				if((y >= p_a[1] && y <= p_b[1]) ||
				   (y >= p_b[1] && y <= p_a[1])) {
					float t = (y - p_a[1]) / (p_b[1] - p_a[1]);
					// find edge-scanline intersection

					if(n_point_num < 2) {
						for(int n = 0; n < 3; ++ n)
							p_segment[n_point_num][n] = p_a[n] + t * (p_b[n] - p_a[n]);
						++ n_point_num;
					} else {
						if(p_segment[0][0] > p_segment[1][0]) {
							for(int n = 0; n < 3; ++ n)
								std::swap(p_segment[0][n], p_segment[1][n]);
						}
						// make sure the first is left and the second is right

						float p_isect[3];
						for(int n = 0; n < 3; ++ n)
							p_isect[n] = p_a[n] + t * (p_b[n] - p_a[n]);
						// calculate the new intersection

						if(p_isect[0] < p_segment[0][0])
							memcpy(p_segment[0], p_isect, 3 * sizeof(float)); // new is left to 0
						else if(p_isect[0] > p_segment[1][0])
							memcpy(p_segment[1], p_isect, 3 * sizeof(float)); // new is right to 1
						// in case the third intersection was found on the same scanline,
						// replace the one lying in the segment of the other two
					}
					// calculate intersection position, add it to the list
				}
			}
			// find intersection of the triangle and the scanline

			if(n_point_num != 2)
				continue;
			// bad intersections

			if(p_segment[0][0] > p_segment[1][0]) {
				for(int n = 0; n < 3; ++ n)
					std::swap(p_segment[0][n], p_segment[1][n]);
			}
			// make sure the first is left and the second is right

			if(int(p_segment[1][0]) < 0 || p_segment[0][0] >= n_width)
				continue;
			// it's too left, or too right

			p_segment[0][2] = 1 / p_segment[0][2];
			p_segment[1][2] = 1 / p_segment[1][2];
			// convert z to 1/z to make its interpolation linear

			float p_delta[3];
			for(int n = 0; n < 3; ++ n)
				p_delta[n] = p_segment[1][n] - p_segment[0][n];
			float f_len = p_delta[0];
			for(int m = 0; m < 3; ++ m)
				p_delta[m] /= f_len;
			// calculate delta coordinates per x-step

			int l = max(0, int(floor(p_segment[0][0])));
			if(p_segment[0][0] != l) {
				float f_offset = l - p_segment[0][0];
				for(int n = 0; n < 3; ++ n)
					p_segment[0][n] += p_delta[n] * f_offset;
			}
			// fixup left point if offscreen

			int r = min(n_width, int(floor(p_segment[1][0])));
			for(; l < r; ++ l) {
				if(p_segment[0][2] < p_z_buffer[l]) {
					p_z_buffer[l] = p_segment[0][2];
					p_color_buffer[l] = n_color;
				}

				for(int n = 0; n < 3; ++ n)
					p_segment[0][n] += p_delta[n];
			}
			// rasterize the segment
		}
		// rasterize the triangle
	}

	/**
	 *	@brief draws a solid-color triangle
	 *
	 *	@param[in] p_vertex is a list of the triangle vertices
	 *	@param[in] n_color is the fill color
	 *
	 *	@note This performs clipping, coordinates outside the bitmap are ok.
	 *
	 *	@todo Write templated variant of this function to support interpolation of more
	 *		coordinates and custom depth test / shading.
	 */
	void DrawTriangle(const float p_vertex[3][2], uint32_t n_color)
	{
		int n_min_y = int(floor(min(p_vertex[0][1], min(p_vertex[1][1], p_vertex[2][1]))));
		int n_max_y = int(floor(max(p_vertex[0][1], max(p_vertex[1][1], p_vertex[2][1]))));
		// find the top / bottom y

		if(n_min_y < 0)
			n_min_y = 0;
		if(n_max_y >= n_height)
			n_max_y = n_height - 1;
		// make sure we don't compute pixels that are not displayed in the end

		uint32_t *p_color_buffer = p_buffer + n_width * n_min_y;
		for(int y = n_min_y; y <= n_max_y; ++ y, p_color_buffer += n_width) {
			int n_point_num = 0;
			float p_segment[2][2];
			for(int i = 0, j = 2; i < 3; j = i ++) {
				const float *p_a = p_vertex[i];
				const float *p_b = p_vertex[j];

				if((y >= p_a[1] && y <= p_b[1]) ||
				   (y >= p_b[1] && y <= p_a[1])) {
					float t = (y - p_a[1]) / (p_b[1] - p_a[1]);
					// find edge-scanline intersection

					if(n_point_num < 2) {
						for(int n = 0; n < 2; ++ n)
							p_segment[n_point_num][n] = p_a[n] + t * (p_b[n] - p_a[n]);
						++ n_point_num;
					} else {
						if(p_segment[0][0] > p_segment[1][0]) {
							for(int n = 0; n < 2; ++ n)
								std::swap(p_segment[0][n], p_segment[1][n]);
						}
						// make sure the first is left and the second is right

						float p_isect[2];
						for(int n = 0; n < 2; ++ n)
							p_isect[n] = p_a[n] + t * (p_b[n] - p_a[n]);
						// calculate the new intersection

						if(p_isect[0] < p_segment[0][0])
							memcpy(p_segment[0], p_isect, 2 * sizeof(float)); // new is left to 0
						else if(p_isect[0] > p_segment[1][0])
							memcpy(p_segment[1], p_isect, 2 * sizeof(float)); // new is right to 1
						// in case the third intersection was found on the same scanline,
						// replace the one lying in the segment of the other two
					}
					// calculate intersection position, add it to the list
				}
			}
			// find intersection of the triangle and the scanline

			if(n_point_num != 2)
				continue;
			// bad intersections

			if(p_segment[0][0] > p_segment[1][0]) {
				for(int n = 0; n < 2; ++ n)
					std::swap(p_segment[0][n], p_segment[1][n]);
			}
			// make sure the first is left and the second is right

			if(int(p_segment[1][0]) < 0 || p_segment[0][0] >= n_width)
				continue;
			// it's too left, or too right

			float p_delta[2];
			for(int n = 0; n < 2; ++ n)
				p_delta[n] = p_segment[1][n] - p_segment[0][n];
			float f_len = p_delta[0];
			for(int m = 0; m < 2; ++ m)
				p_delta[m] /= f_len;
			// calculate delta coordinates per x-step

			int l = max(0, int(floor(p_segment[0][0])));
			if(p_segment[0][0] != l) {
				float f_offset = l - p_segment[0][0];
				for(int n = 0; n < 2; ++ n)
					p_segment[0][n] += p_delta[n] * f_offset;
			}
			// fixup left point if offscreen

			int r = min(n_width, int(floor(p_segment[1][0])));
			for(; l < r; ++ l) {
				p_color_buffer[l] = n_color;

				for(int n = 0; n < 3; ++ n)
					p_segment[0][n] += p_delta[n];
			}
			// rasterize the segment
		}
		// rasterize the triangle
	}

	/**
	 *	@brief applies a simple convolution filter
	 *
	 *	@tparam CFilterInside is implementation of the filter for the interior pixels
	 *		(can ommit out of bounds checking)
	 *	@tparam CFilterBorder is implementation of the filter for the border pixels
	 *
	 *	@param[in] r_t_src is source image
	 *	@param[in] n_filter_width is filter width, in pixels (a radius rather than diameter)
	 *	@param[in] n_filter_height is filter height, in pixels (a radius rather than diameter)
	 *	@param[in] filter_inside is instance of the filter for the interior pixels
	 *	@param[in] filter_border is instance of the filter for the border pixels
	 *
	 *	@note Before calling, this bitmap must be allocated to the same size as r_t_src.
	 *	@note This function is not cache friendly, as it does not perform any tiling.
	 */
	template <class CFilterInside, class CFilterBorder>
	void FilterLoop(const TBmp &r_t_src, int n_filter_width, int n_filter_height,
		CFilterInside filter_inside, CFilterBorder filter_border)
	{
		_ASSERTE(n_width == r_t_src.n_width && n_height == r_t_src.n_height);
		if(n_width <= 2 * n_filter_width || n_height <= 2 * n_filter_height) {
			uint32_t *p_dest = p_buffer;
			const uint32_t *p_src = r_t_src.p_buffer;
			for(int y = 0, w = n_width, h = n_height; y < h; ++ y) {
				for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_border(p_src, x, y, w, h);
			}
			// the image is small or the filter is big; all the pixels are on the border
		} else {
			const int w = n_width, h = n_height;
			// antialiass

			uint32_t *p_dest = p_buffer;
			const uint32_t *p_src = r_t_src.p_buffer;

			for(int y = 0; y < n_filter_height; ++ y) {
				for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_border(p_src, x, y, w, h);
			}
			// top of the border

			for(int y = n_filter_height, ex = w - n_filter_width,
			   ey = h - n_filter_height; y < ey; ++ y) {
				for(int x = 0; x < n_filter_width; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_border(p_src, x, y, w, h);
				// left side (border)

				for(int x = n_filter_width; x < ex; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_inside(p_src, w);
				// filter inside of the image

				for(int x = ex; x < w; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_border(p_src, x, y, w, h);
				// right side (border)
			}

			for(int y = h - n_filter_height; y < h; ++ y) {
				for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
					*p_dest = filter_border(p_src, x, y, w, h);
			}
			// bottom of the border

			_ASSERTE(p_dest == p_buffer + w * h);
			_ASSERTE(p_src == r_t_src.p_buffer + w * h);
			// make sure we increment it correctly
		}
	}

	/**
	 *	@brief fixed size filter loop
	 *
	 *	@tparam n_filter_width is filter width, in pixels (a radius rather than diameter)
	 *	@tparam n_filter_height is filter height, in pixels (a radius rather than diameter)
	 */
	template <int n_filter_width, int n_filter_height>
	class CConstFilterLoop {
	public:
		/**
		 *	@brief applies a simple convolution filter
		 *
		 *	@tparam CFilterInside is implementation of the filter for the interior pixels
		 *		(can ommit out of bounds checking)
		 *	@tparam CFilterBorder is implementation of the filter for the border pixels
		 *
		 *	@param[in] r_t_dest is destination image (must be the same size as r_t_src)
		 *	@param[in] r_t_src is source image
		 *	@param[in] filter_inside is instance of the filter for the interior pixels
		 *	@param[in] filter_border is instance of the filter for the border pixels
		 *
		 *	@note Before calling, r_t_dest must be allocated to the same size as r_t_src.
		 *	@note This function is not cache friendly, as it does not perform any tiling.
		 */
		template <class CFilterInside, class CFilterBorder>
		static void FilterLoop(TBmp &r_t_dest, const TBmp &r_t_src,
			CFilterInside filter_inside, CFilterBorder filter_border)
		{
			_ASSERTE(r_t_dest.n_width == r_t_src.n_width && r_t_dest.n_height == r_t_src.n_height);
			if(r_t_dest.n_width <= 2 * n_filter_width || r_t_dest.n_height <= 2 * n_filter_height) {
				uint32_t *p_dest = r_t_dest.p_buffer;
				const uint32_t *p_src = r_t_src.p_buffer;
				for(int y = 0, w = r_t_dest.n_width, h = r_t_dest.n_height; y < h; ++ y) {
					for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_border(p_src, x, y, w, h);
				}
				// the image is small or the filter is big; all the pixels are on the border
			} else {
				const int w = r_t_dest.n_width, h = r_t_dest.n_height;
				// antialiass

				uint32_t *p_dest = r_t_dest.p_buffer;
				const uint32_t *p_src = r_t_src.p_buffer;

				for(int y = 0; y < n_filter_height; ++ y) {
					for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_border(p_src, x, y, w, h);
				}
				// top of the border

				for(int y = n_filter_height, ex = w - n_filter_width,
				   ey = h - n_filter_height; y < ey; ++ y) {
					for(int x = 0; x < n_filter_width; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_border(p_src, x, y, w, h);
					// left side (border)

					for(int x = n_filter_width; x < ex; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_inside(p_src, w);
					// filter inside of the image

					for(int x = ex; x < w; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_border(p_src, x, y, w, h);
					// right side (border)
				}

				for(int y = h - n_filter_height; y < h; ++ y) {
					for(int x = 0; x < w; ++ x, ++ p_dest, ++ p_src)
						*p_dest = filter_border(p_src, x, y, w, h);
				}
				// bottom of the border
			}
		}
	};

	/**
	 *	@brief common filter kernel implementations
	 */
	class CBasicFilterKernels {
	public:
		/**
		 *	@brief dummy border filter implementation
		 *
		 *	@param[in] p_src is pointer to the current pixel (unused)
		 *	@param[in] x is horizontal coordinate of the current pixel (unused)
		 *	@param[in] y is vertical coordinate of the current pixel (unused)
		 *	@param[in] w is width of the input image, in pixels (unused)
		 *	@param[in] h is height of the input image, in pixels (unused)
		 *
		 *	@return Returns black, with full alpha.
		 */
		static uint32_t n_DummyFilterBorder(const uint32_t *UNUSED(p_src),
			int UNUSED(x), int UNUSED(y), int UNUSED(w), int UNUSED(h))
		{
			return 0xff000000U;
		}

		/**
		 *	@brief Roberts-Cross filter implementation
		 *
		 *	@param[in] p_src is pointer to the current pixel
		 *	@param[in] w is width of the input image, in pixels
		 *
		 *	@return Returns the filtered grayscale values.
		 */
		static uint32_t n_RobertCross_Gray(const uint32_t *p_src, int w)
		{
			int n_horz = abs(int(*p_src & 0xff) - int(p_src[-1] & 0xff));
			int n_vert = abs(int(*p_src & 0xff) - int(p_src[-w] & 0xff));
			int n_gray = abs(n_horz) + abs(n_vert);
			n_gray = min(255, n_gray);
			return 0xff000000U | n_gray | (n_gray << 8) | (n_gray << 16);
		}

		/**
		 *	@brief single-pass Sobel filter implementation
		 *
		 *	@param[in] p_src is pointer to the current pixel
		 *	@param[in] w is width of the input image, in pixels
		 *
		 *	@return Returns the filtered grayscale values.
		 */
		static uint32_t n_Sobel_Gray(const uint32_t *p_src, int w)
		{
			int n_horz =
				-1 * (p_src[-1 - w] & 0xff) +
				-2 * (p_src[-1 - 0] & 0xff) +
				-1 * (p_src[-1 + w] & 0xff) +
				+1 * (p_src[+1 - w] & 0xff) +
				+2 * (p_src[+1 - 0] & 0xff) +
				+1 * (p_src[+1 + w] & 0xff);
			int n_vert =
				-1 * (p_src[-1 - w] & 0xff) +
				-2 * (p_src[-0 - w] & 0xff) +
				-1 * (p_src[+1 - w] & 0xff) +
				+1 * (p_src[-1 + w] & 0xff) +
				+2 * (p_src[-0 + w] & 0xff) +
				+1 * (p_src[+1 + w] & 0xff);
			int n_gray = abs(n_horz) + abs(n_vert);
			n_gray = min(255, n_gray);
			return 0xff000000U | n_gray | (n_gray << 8) | (n_gray << 16);
		}

		/**
		 *	@brief 3x3 Gaussian-like filter implementation
		 *
		 *	@param[in] p_src is pointer to the current pixel
		 *	@param[in] w is width of the input image, in pixels
		 *
		 *	@return Returns the filtered grayscale values.
		 */
		static uint32_t n_Unsharp_Mask33(const uint32_t *p_src, int w)
		{
			int n_gray =
				(1 * (p_src[-1 - w] & 0xff) +
				2 * (p_src[-1 - 0] & 0xff) +
				1 * (p_src[-1 + w] & 0xff) +
				2 * (p_src[ 0 - w] & 0xff) +
				4 * (p_src[ 0 - 0] & 0xff) +
				2 * (p_src[ 0 + w] & 0xff) +
				1 * (p_src[+1 - w] & 0xff) +
				2 * (p_src[+1 - 0] & 0xff) +
				1 * (p_src[+1 + w] & 0xff)) / 16;
			return 0xff000000U | n_gray | (n_gray << 8) | (n_gray << 16);
		}
	};

	/**
	 *	@brief RGB to grayscale conversion helper
	 */
	struct TGrayscaleConv {
		/**
		 *	@brief function operator
		 *	@param[in,out] r_n_pixel is pixel to be converted to grayscale
		 *	@note This preserves alpha.
		 */
		inline void operator ()(uint32_t &r_n_pixel) const
		{
			uint32_t c = r_n_pixel;
			int r = (c >> 16) & 0xff;
			int g = (c >> 8) & 0xff;
			int b = c & 0xff;
			int n_grey = (r * int(.299f * 0x10000) + g * int(.578f * 0x10000) + b * int(.114f * 0x10000)) >> 16;
			r_n_pixel = (r_n_pixel & 0xff000000U) | (n_grey << 16) | (n_grey << 8) | n_grey;
		}
	};

	/**
	 *	@brief alpha fill helper
	 */
	struct TMakeOpaque {
		/**
		 *	@brief function operator
		 *	@param[in,out] r_n_pixel is pixel to be opaqued
		 */
		inline void operator ()(uint32_t &r_n_pixel) const
		{
			r_n_pixel |= 0xff000000U;
		}
	};

	/**
	 *	@brief RGB to BGR conversion helper
	 */
	struct TRGB_BGR_Swap {
		/**
		 *	@brief function operator
		 *	@param[in,out] r_n_pixel is pixel to be converted to BGR
		 *	@note This preserves alpha.
		 */
		inline void operator ()(uint32_t &r_n_pixel) const
		{
			uint32_t n_color = r_n_pixel;
			r_n_pixel = (n_color & 0xff00ff00U) | ((n_color & 0xff0000) >> 16) | ((n_color & 0x0000ff) << 16);
		}
	};

	/**
	 *	@brief a simple interpolator object for rasterizing lines
	 */
    class CInterpolator {
	public:
		typedef int TFixedPoint; /**< @brief this interpolator should use fixed-point numbers for increased accuracy */

    protected:
        TFixedPoint m_y; /**< @brief current interpolated value */
        int m_n_length; /**< @brief length of the interpolation domain */
        TFixedPoint m_n_slope; /**< @brief interpolated line slope */
        TFixedPoint m_n_error; /**< @brief error accumulator */
        TFixedPoint m_n_remainder; /**< @brief error increase per interpolation step */

    public:
		/**
		 *	@brief default constructor
		 *
		 *	@param[in] n_y0 is the interpolated value at the beginning of the domain
		 *	@param[in] n_y1 is the interpolated value at the end of the domain
		 *	@param[in] n_length is length of the domain
		 */
        inline CInterpolator(TFixedPoint n_y0, TFixedPoint n_y1, int n_length)
			:m_y(n_y0), m_n_length(max(1, n_length)), m_n_slope((n_y1 - n_y0) / m_n_length),
            m_n_error((n_y1 - n_y0) % m_n_length), m_n_remainder(m_n_error)
        {
            if(m_n_error <= 0) {
                m_n_error += n_length;
                m_n_remainder += n_length;
                -- m_n_slope;
            }
			// fix the rounding direction for negative values

            m_n_error -= n_length; // !!
        }

		/**
		 *	@brief calculates interpolated value at the next position
		 */
        inline void operator ++()
        {
            m_y += m_n_slope;
            if((m_n_error += m_n_remainder) > 0) {
                m_n_error -= m_n_length;
                ++ m_y;
            }
			// DDA
        }

		/**
		 *	@brief gets interpolated value
		 *	@return Returns the current interpolated value.
		 */
        inline TFixedPoint y() const
		{
			return m_y;
		}
    };
};

#endif // !__BITMAP_STRUCTURE_INCLUDED
