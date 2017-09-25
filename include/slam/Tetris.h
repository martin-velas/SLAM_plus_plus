/*
								+----------------------------------+
								|                                  |
								| *** Tetris pattern generator *** |
								|                                  |
								|  Copyright (c) -tHE SWINe- 2012  |
								|                                  |
								|             Tetris.h             |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __TETRIS_INCLUDED
#define __TETRIS_INCLUDED

/**
 *	@file include/slam/Tetris.h
 *	@brief tetris
 *	@author -tHE SWINe-
 *	@date 2012
 *
 *	This is used to test block matrix storage. Other alternatives were
 *	Conway's Life or some variants of ants and worms. I generaly prefer
 *	to avoid having insects in my code. And Life is boring and repetitive.
 *
 *	It is possible to use the tetris_main() function to run the benchmark.
 *
 *	@note This contains some functions and data, it should be only included
 *		from a single .cpp file (me being lazy to split it to .cpp and .h).
 */

#if !defined(_DEBUG) && (defined(_WIN32) || defined(_WIN64))
/**
 *	@def __TIMER_POWERED_AI
 *	@brief if defined, AI is called from inside a timer (otherwise it's caled when
 *		the message loop is idle, giving much faster animation)
 *	@note Debug always uses infinite-loop AI, debug checks slow the animation down sufficiently.
 */
//#define __TIMER_POWERED_AI
#endif // !_DEBUG && (_WIN32 || _WIN64)

/**
 *	@def __TETRIS_EASY_TILES
 *	@brief enables the use of easy tetris tiles
 */
#define __TETRIS_EASY_TILES

/**
 *	@def __TETRIS_MEDIUM_TILES
 *	@brief enables the use of medium tetris tiles
 */
#define __TETRIS_MEDIUM_TILES

/**
 *	@def __TETRIS_HARD_TILES
 *	@brief enables the use of hard tetris tiles (unplayable)
 */
//#define __TETRIS_HARD_TILES

/**
 *	@brief tetris game class (graphics is win32-only)
 *	@note This class is a singleton in win32.
 */
class CTetrisGame {
protected:
	int m_n_field_width; /**< @brief playing field width, in blocks */
	int m_n_field_height; /**< @brief playing field height, in blocks */
	int m_n_window_width; /**< @brief window width, in pixels */
	int m_n_window_height; /**< @brief window height, in pixels */
	int m_n_scalar_size; /**< @brief size of a rasterized scalar element, in pixels */
	std::vector<bool> m_field; /**< @brief playing field, represented by a bit array */
	Eigen::MatrixXd m_block; /**< @brief playing field, represented by a block matrix */
#if defined(_WIN32) || defined(_WIN64)
	HWND m_h_wnd; /**< @brief handle to the game window (windows only) */
	static CTetrisGame *m_p_instance; /**< @brief instance of the tetris game (it's a singleton, windows only) */
#endif // _WIN32 || _WIN64
	int m_n_dropping_tile_type; /**< @brief type of the current active tile */
	int m_n_dropping_tile_pos_x; /**< @brief horizontal position of the current active tile */
	int m_n_dropping_tile_pos_y; /**< @brief vertical position of the current active tile */
	int m_n_dropping_tile_angle; /**< @brief angle of the current active tile */
	int m_n_score; /**< @brief current score */
	bool m_b_game_over; /**< @brief game over flag */
	bool m_b_AI_game; /**< @brief AI game flag (ignores user input if set) */
	TBmp *m_p_bitmap; /**< @brief cached storage */
	bool m_b_vertical_fill; /**< @brief vertical fill flag (otherwise the matrix is filled horizontaly) */
	bool m_b_backwards_fill; /**< @brief backwards fill flag (otherwise the matrix is filled from upper left) */
	bool m_b_fill_tile_first; /**< @brief fill the active tile first flag (otherwise the old tiles in playing field are filled first) */
	bool m_b_swap_loops; /**< @brief row major fill flag (similar to m_b_vertical_fill, but does not flip the game as well) */

public:
	/**
	 *	@brief default constructor
	 *
	 *	@param[in] b_vertical_fill is vertical fill flag (otherwise the matrix is filled horizontaly)
	 *	@param[in] b_backwards_fill is backwards fill flag (otherwise the matrix is filled from upper left)
	 *	@param[in] b_fill_tile_first is fill the active tile first flag
	 *		(otherwise the old tiles in playing field are filled first)
	 *	@param[in] b_row_major_fill is row major fill flag (similar to b_vertical_fill,
	 *		but does not flip the game as well)
	 *	@param[in] n_field_width is playing field width, in blocks
	 *	@param[in] n_field_height is playing field height, in blocks
	 *	@param[in] n_block_cols is number of block columns, in scalars
	 *	@param[in] n_block_rows is number of block rows, in scalars
	 *	@param[in] n_scalar_size is size of a rasterized scalar element, in pixels
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	CTetrisGame(bool b_vertical_fill, bool b_backwards_fill, bool b_fill_tile_first,
		bool b_row_major_fill, int n_field_width, int n_field_height,
		int n_block_cols = 3, int n_block_rows = 3, int n_scalar_size = 5) // throw(std::bad_alloc)
		:m_n_field_width(n_field_width), m_n_field_height(n_field_height),
		m_n_window_width(n_field_width * n_block_cols * (n_scalar_size - 1) + 1),
		m_n_window_height(m_n_field_height * n_block_rows * (n_scalar_size - 1) + 1),
		m_n_scalar_size(n_scalar_size), m_block(n_block_rows, n_block_cols),
#if defined(_WIN32) || defined(_WIN64)
		m_h_wnd(0),
#endif // _WIN32 || _WIN64
		m_n_dropping_tile_type(0), m_n_dropping_tile_pos_x(n_field_width / 2 - 2),
		m_n_dropping_tile_pos_y(-4), m_n_dropping_tile_angle(0), m_n_score(0),
		m_b_game_over(false), m_b_AI_game(false), m_p_bitmap(0),
		m_b_vertical_fill(b_vertical_fill), m_b_backwards_fill(b_backwards_fill),
		m_b_fill_tile_first(b_fill_tile_first), m_b_swap_loops(b_row_major_fill)
	{
		if(b_vertical_fill)
			std::swap(m_n_field_width, m_n_field_height);

		m_block.setConstant(M_PI);

		m_field.resize(m_n_field_width * m_n_field_height, false);
#if defined(_WIN32) || defined(_WIN64)
		_ASSERTE(!m_p_instance);
		m_p_instance = this;
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief destructor; deleted memory allocated by the game
	 */
	~CTetrisGame()
	{
		if(m_p_bitmap)
			m_p_bitmap->Delete();
#if defined(_WIN32) || defined(_WIN64)
		_ASSERTE(m_p_instance == this);
		m_p_instance = 0;
#endif // _WIN32 || _WIN64
	}

	/**
	 *	@brief creates bitmask for a tile
	 *
	 *	@param[in] n_type is tile type (wraps around, can be any number)
	 *	@param[in] n_rotation is rotation, in multiples of pi half radians
	 *
	 *	@return Returns bitmask for the specified tile.
	 */
	static uint16_t n_Tile(int n_type, int n_rotation)
	{
		const char *p_tiles[] = {
#ifdef __TETRIS_EASY_TILES
			"    "
			" ** "
			" ** "
			"    ",

			"    "
			" *  "
			"*** "
			"    ",

			" *  "
			" *  "
			" *  "
			" *  ",

			"  * "
			" ** "
			" *  "
			"    ",

			" *  "
			" ** "
			"  * "
			"    ",

			" *  "
			" *  "
			" ** "
			"    ",

			"  * "
			"  * "
			" ** "
			"    ",
#endif // __TETRIS_EASY_TILES

#ifdef __TETRIS_MEDIUM_TILES
			" *  "
			"*** "
			" *  "
			"    ",

			"    "
			"  * "
			" ** "
			"    ",

			"    "
			" *  "
			" ** "
			"    ",

			"   *"
			"****"
			"*   "
			"    ",

			"    "
			" * *"
			" ***"
			"    ",

			"    "
			"*** "
			"* * "
			"    ",

			"    "
			"    "
			"  * "
			"    ",

			"    "
			"  * "
			"  * "
			"    ",

			"  * "
			"  * "
			"  * "
			"    ",

			" ** "
			" *  "
			"  * "
			"    ",

			"  * "
			" *  "
			" ** "
			"    ",

			"    "
			"  * "
			" *  "
			"    ",
#endif // __TETRIS_MEDIUM_TILES

#ifdef __TETRIS_HARD_TILES
			" *  "
			" *  "
			" ***"
			"    ",

			" *  "
			" ** "
			"  **"
			"    ",

			"  * "
			" ** "
			"  * "
			"  * ",

			"  **"
			" ** "
			"  * "
			"    ",

			" ** "
			"  **"
			"  * "
			"    ",

			" ***"
			"  * "
			"  * "
			"    ",

			"  * "
			"  * "
			" ** "
			" *  ",

			" ** "
			" ** "
			" *  "
			" *  ",

			" ** "
			" *  "
			" *  "
			" ** ",

			" *  "
			" ** "
			" ** "
			" *  ",

			" *  "
			"**  "
			" ** "
			" *  ",

			" ** "
			" *  "
			"**  "
			" *  ",

			" *  "
			"*** "
			" *  "
			" *  ",

			"  * "
			" ** "
			" *  "
			" ** ",

			"    "
			" ***"
			" * *"
			" *  ",

			"  * "
			" ***"
			" *  "
			" *  ",

			"    "
			"  * "
			" ***"
			" ** ",

			"    "
			"*   "
			"**  "
			"*** ",

			"  * "
			"*** "
			" *  "
			" *  ",

			"   *"
			"  **"
			" ** "
			" *  ",

			" * *"
			" ** "
			" *  "
			" *  ",

			"* * "
			"*** "
			" *  "
			"    ",
#endif // __TETRIS_HARD_TILES
		};
		const size_t n_tile_type_num = sizeof(p_tiles) / sizeof(p_tiles[0]);
		// table of known tetris tiles

		int r00, r10, r20;
		int r01, r11, r21;
		n_rotation %= 4;
		if(n_rotation == 0) {
			r00 = 1; r10 = 0; r20 = 0;
			r01 = 0; r11 = 1; r21 = 0;
		} else if(n_rotation == 1) {
			r00 = 0; r10 = -1; r20 = 3;
			r01 = 1; r11 = 0; r21 = 0;
		} else if(n_rotation == 2) {
			r00 = -1; r10 = 0; r20 = 3;
			r01 = 0; r11 = -1; r21 = 3;
		} else if(n_rotation == 3) {
			r00 = 0; r10 = 1; r20 = 0;
			r01 = -1; r11 = 0; r21 = 3;
		}
		// build the rotation matrix

		uint16_t n_tile = 0;
		n_type %= n_tile_type_num;
		for(int y = 0; y < 4; ++ y) {
			for(int x = 0; x < 4; ++ x) {
				int n_tx = x * r00 + y * r10 + r20;
				int n_ty = x * r01 + y * r11 + r21;
				n_tile |= ((p_tiles[n_type][n_tx + n_ty * 4] == '*')? 1 : 0) << (x + 4 * y);
			}
		}
		// build the rotated tile in binary form

		return n_tile;
	}

	/**
	 *	@brief runs the game
	 *	@param[in] b_AI_play is artificial inteligence flag
	 *	@return Returns true on success, false on failure.
	 */
	bool Run(bool b_AI_play)
	{
		m_b_AI_game = b_AI_play;
		Restart();

#if defined(_WIN32) || defined(_WIN64)
		const char *p_s_class_name = "Tetris-Window-Class";
		const char *p_s_window_name = "Sparse Tetris";

		WNDCLASSEX t_wnd_class;
		t_wnd_class.cbSize = sizeof(WNDCLASSEX);
		t_wnd_class.style = CS_HREDRAW | CS_VREDRAW;
		t_wnd_class.lpfnWndProc = &_WndProc;
		t_wnd_class.cbClsExtra = 0;
		t_wnd_class.cbWndExtra = 0;
		t_wnd_class.hInstance = GetModuleHandle(NULL);
		t_wnd_class.hIcon = NULL;
		t_wnd_class.hCursor = LoadCursor(0, IDC_ARROW);
		t_wnd_class.hbrBackground = NULL;
		t_wnd_class.lpszMenuName = 0;
		t_wnd_class.lpszClassName = p_s_class_name;
		t_wnd_class.hIconSm = NULL;
		if(!RegisterClassEx(&t_wnd_class))
			return false;
		// create window class

		RECT t_rect;
		t_rect.left = 0;
		t_rect.top = 0;
		t_rect.right = m_n_window_width;
		t_rect.bottom = m_n_window_height;
		AdjustWindowRectEx(&t_rect, WS_OVERLAPPEDWINDOW,
			FALSE, WS_EX_APPWINDOW | WS_EX_WINDOWEDGE | WS_EX_TOOLWINDOW);
		int n_outer_width = t_rect.right - t_rect.left,
			n_outer_height = t_rect.bottom - t_rect.top;
		// adjust window rect, based on window style

		if(!(m_h_wnd = CreateWindowEx(WS_EX_TOOLWINDOW,
		   p_s_class_name, p_s_window_name, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
		   n_outer_width, n_outer_height, NULL, NULL, GetModuleHandle(NULL), NULL)))
			return false;
		// create the window

		ShowWindow(m_h_wnd, SW_SHOW);
		UpdateWindow(m_h_wnd);
		// show window (first frame is drawn)

#ifdef __TIMER_POWERED_AI
		if(b_AI_play)
			SetTimer(m_h_wnd, 1, 1, NULL);
#endif // __TIMER_POWERED_AI
		SetTimer(m_h_wnd, 0, 500, NULL);
		// set timer for animations

		MSG t_msg;
#ifdef __TIMER_POWERED_AI
		while(GetMessage(&t_msg, /*m_h_wnd*/0, 0, 0) > 0) { // avoid using h_wnd in message loop, windows 7 will not repaint the window properly then ...
			TranslateMessage(&t_msg);
			DispatchMessage(&t_msg);
		}
#else // __TIMER_POWERED_AI
		if(b_AI_play) {
			while(IsWindow(m_h_wnd)) {
				if(PeekMessage(&t_msg, /*m_h_wnd*/0, 0, 0, PM_REMOVE)) {
					TranslateMessage(&t_msg);
					DispatchMessage(&t_msg);
				} else {
					if(!m_b_game_over) {
						AI_NextStep();
						InvalidateRect(m_h_wnd, 0, false);
					}
				}
			}
		} else {
			while(GetMessage(&t_msg, /*m_h_wnd*/0, 0, 0) > 0) { // avoid using h_wnd in message loop, windows 7 will not repaint the window properly then ...
				TranslateMessage(&t_msg);
				DispatchMessage(&t_msg);
			}
		}
#endif // __TIMER_POWERED_AI
		// process messages

		UnregisterClass(t_wnd_class.lpszClassName, GetModuleHandle(NULL));
		// unregister the class so it can be registered again

		m_h_wnd = NULL;
		// this is no longer valid
#else // _WIN32 || _WIN64
		if(!b_AI_play)
			return false; // only AI can play on linux / mac
		while(!m_b_game_over)
			AI_NextStep();
		// play a single AI game on linux / mac
#endif // _WIN32 || _WIN64

		return true;
	}

	/**
	 *	@brief does simple block matrix tests with single blocks
	 */
	static void Do_TetrisBlocksTest()
	{
		const char *p_tiles[] = {
			"**  "
			"**  "
			"    "
			"    ",
			" *  "
			"*** "
			"    "
			"    ",
			"*   "
			"*   "
			"*   "
			"*   ",
			" *  "
			"**  "
			"*   "
			"    ",
			"*   "
			"**  "
			" *  "
			"    ",
			"*   "
			"*   "
			"**  "
			"    ",
			" *  "
			" *  "
			"**  "
			"    ",
			" *  "
			"*** "
			" *  "
			"    ",
			" *  "
			"**  "
			"    "
			"    ",
			"*   "
			"**  "
			"    "
			"    ",
			"   *"
			"****"
			"    "
			"    ",
			"   *"
			"****"
			"*   "
			"    "
		};

		Eigen::MatrixXd m(3, 3);
		CUberBlockMatrix bm;
		bm.Append_Block(m, 0, 0);
		bm.Rasterize("debug\\block_matrix_test0.tga");
		bm.Append_Block(m, 3, 3);
		bm.Rasterize("debug\\block_matrix_test1.tga");
		bm.Append_Block(m, 3, 0);
		bm.Rasterize("debug\\block_matrix_test2.tga");
		// very basic test

		for(size_t i = 0, n = sizeof(p_tiles) / sizeof(p_tiles[0]); i < n; ++ i) {
			CUberBlockMatrix bm;
			size_t n_block_num = 0;
			for(size_t n_row = 0; n_row < 4; ++ n_row) {
				for(size_t n_col = 0; n_col < 4; ++ n_col) {
					if(p_tiles[i][n_col + 4 * n_row] != ' ') {
						bm.Append_Block(m, n_row * 3, n_col * 3);
						++ n_block_num;

						char p_s_filename[256];
						sprintf(p_s_filename, "debug\\block_matrix_tetris_test%02d_step%02d.tga", i, n_block_num);
						bm.Rasterize(p_s_filename);
					}
				}
			}
			char p_s_filename[256];
			sprintf(p_s_filename, "debug\\block_matrix_tetris_test%02d_final.tga", i);
			bm.Rasterize(p_s_filename);
		}
		// tetris test
	}

protected:
	/**
	 *	@brief resets the inner game state
	 */
	void Restart()
	{
		m_n_dropping_tile_pos_x = m_n_field_width / 2 - 2;
		m_n_dropping_tile_pos_y = -4;
		m_n_dropping_tile_angle = 0;
		m_n_score = 0;
		m_b_game_over = false;
		srand(int(time(NULL)));
		m_n_dropping_tile_type = rand();
		m_field.clear();
		_ASSERTE(m_field.capacity() >= unsigned(m_n_field_width * m_n_field_height));
		m_field.resize(m_n_field_width * m_n_field_height, false);
		// pick a truly random tile
	}

	/**
	 *	@brief detects if the active tile would collide at given coordinates with given angle
	 *
	 *	@param[in] n_dx is horizontal delta position from the current active tile position
	 *	@param[in] n_dy is vertical delta position from the current active tile position
	 *	@param[in] n_d_angle is delta angle from the current active tile angle
	 *
	 *	@return Returns true if the tile would have collided, otherwise returns false.
	 */
	bool b_TileColided(int n_dx, int n_dy, int n_d_angle = 0)
	{
		return b_TileColided2(n_Tile(m_n_dropping_tile_type, m_n_dropping_tile_angle + n_d_angle), n_dx, n_dy);
	}

	/**
	 *	@brief detects if a tile would collide at given coordinates with given angle
	 *
	 *	@param[in] n_tile is tile bit mask (as the ones obtained by calling n_Tile())
	 *	@param[in] n_dx is horizontal delta position from the current active tile position
	 *	@param[in] n_dy is vertical delta position from the current active tile position
	 *
	 *	@return Returns true if the tile would have collided, otherwise returns false.
	 */
	bool b_TileColided2(uint16_t n_tile, int n_dx, int n_dy)
	{
		for(int j = 0; j < 4; ++ j) {
			for(int i = 0; i < 4; ++ i) {
				if(((n_tile >> (i + 4 * j)) & 1) != 0) {
					int n_col = i + m_n_dropping_tile_pos_x + n_dx;
					int n_row = j + m_n_dropping_tile_pos_y + n_dy;
					if(n_col < 0 || n_col >= m_n_field_width)
						return true; // the tile smashed into one of the walls
					if(n_row >= m_n_field_height)
						return true; // the tile sits on the floor
					if(n_col >= 0 && n_col < m_n_field_width &&
					   n_row >= 0 && n_row < m_n_field_height &&
					   m_field[n_col + m_n_field_width * n_row])
						return true; // the tile collided with the geometry
				}
			}
		}

		return false;
	}

	/**
	 *	@brief runs the AI to decide the next move (left / right / down)
	 */
	void AI_NextStep()
	{
		uint16_t p_tile[4] = {n_Tile(m_n_dropping_tile_type, 0),
			n_Tile(m_n_dropping_tile_type, 1), n_Tile(m_n_dropping_tile_type, 2),
			n_Tile(m_n_dropping_tile_type, 3)};
		// get all the rotations of the tile

		int n_best_pos = 0;
		int n_best_angle = 0;
		int n_best_inacc = INT_MAX;
		int n_best_obstructed = INT_MAX;
		int n_best_low = m_n_field_height;
		int n_best_high = -4;

		int n_best_score = INT_MIN;

		int n_fill = 0;
		for(int i = 0; i < m_n_field_height; ++ i) {
			bool b_have_fill = false;
			for(int j = 0; j < m_n_field_width; ++ j) {
				if(m_field[j + i * m_n_field_width]) {
					b_have_fill = true;
					break;
				}
			}
			if(b_have_fill) {
				n_fill = i;
				break;
			}
		}
		// see where is the highest block to decide strategy

		for(int n_angle = 0; n_angle < 4; ++ n_angle) {
			for(int n_dx = -m_n_field_width; n_dx <= m_n_field_width; ++ n_dx) {
				if(b_TileColided2(p_tile[n_angle], n_dx, 0))
					continue; // can't shift here
				for(int n_dy = 1; n_dy < m_n_field_height; ++ n_dy) {
					if(b_TileColided2(p_tile[n_angle], n_dx, n_dy)) {
						int n_low = m_n_field_height;
						int n_high = -4;
						for(int j = 0; j < 4; ++ j) {
							for(int i = 0; i < 4; ++ i) {
								int n_col = i + m_n_dropping_tile_pos_x + n_dx;
								int n_row = j + m_n_dropping_tile_pos_y + n_dy - 1;
								if(((p_tile[n_angle] >> (i + 4 * j)) & 1) != 0) {
									if(n_row < n_low)
										n_low = n_row;
									if(n_high < n_row)
										n_high = n_row;
								}
							}
						}
						// find high/low coordinates of the tile ...

						int n_obstructed = 0;
						int n_inacc = 0;
						for(int i = 0; i < 4; ++ i) {
							bool b_had_block = false;
							bool b_had_nonzero = false;
							for(int j = 0; j < 8; ++ j) { // look deep
								if(!b_had_nonzero && j < 4 && ((p_tile[n_angle] >> (i + 4 * j)) & 1) != 0) {
									b_had_nonzero = true;
									continue;
								}
								if(b_had_nonzero && (j >= 4 || ((p_tile[n_angle] >> (i + 4 * j)) & 1) == 0)) {
									// we are in an empty space under a tile

									int n_col = i + m_n_dropping_tile_pos_x + n_dx;
									int n_row = j + m_n_dropping_tile_pos_y + n_dy - 1;
									if(n_col >= 0 && n_col < m_n_field_width &&
									   n_row >= 0 && n_row < m_n_field_height &&
									   m_field[n_col + m_n_field_width * n_row]) {
										b_had_block = true;
										continue;
									}
									// there is a block that is in the game field. stop counting ...

									if(n_row >= m_n_field_height)
										break;
									// we hit the floor. stop counting ...

									if(!b_had_block) {
										++ n_inacc;
										// we are in an empty space that could be potentially
										// filled, but is inaccessible due to tile placement
									} else {
										++ n_obstructed;
										// there is an empty space below. might be better
										// not to pile stuff in here in case there is another option
									}
								}
							}
						}
						// find number of inaccessible tiles due to placing this one here,
						// and a number of obstructed tiles below it

						if(n_fill < m_n_field_height / 2) {
							int n_score = n_inacc * -250 + n_best_obstructed * -50 +
								n_high * 100 + n_low * 75;
							// too many tiles; gotta compromise ...

							if(n_score > n_best_score || (n_score == n_best_score &&
							   n_dx == 0 && n_angle == m_n_dropping_tile_angle)) {
								n_best_score = n_score;
								n_best_angle = n_angle;
								n_best_pos = n_dx;
							}
						} else {
							if(n_best_inacc > n_inacc || (n_best_inacc == n_inacc &&
							   (n_best_high < n_high || (n_best_high == n_high &&
							   (n_best_obstructed > n_obstructed || (n_best_obstructed == n_obstructed &&
							   (n_best_low < n_low || (n_best_low == n_low &&
							   n_dx == 0 && n_angle == m_n_dropping_tile_angle)))))))) {
								n_best_obstructed = n_obstructed;
								n_best_inacc = n_inacc;
								n_best_low = n_low;
								n_best_high = n_high;
								n_best_angle = n_angle;
								n_best_pos = n_dx;
							}
						}
						// calculate score, remember better position

						break;
					}
				}
			}
		}
		// let all the rotations fall at all the positions, see what falls the deepest

		if(n_best_angle != m_n_dropping_tile_angle) {
			if(!b_TileColided(0, 0, 1))
				m_n_dropping_tile_angle = (m_n_dropping_tile_angle + 1) % 4;
			else if(!b_TileColided(1, 0, 1))
				++ m_n_dropping_tile_pos_x;
			else if(!b_TileColided(-1, 0, 1))
				-- m_n_dropping_tile_pos_x;
		} else if(n_best_pos) {
			if(!b_TileColided(((n_best_pos < 0)? -1 : 1), 0))
				m_n_dropping_tile_pos_x += ((n_best_pos < 0)? -1 : 1);
		} else {
			if(!b_TileColided(0, 1)) {
				++ m_n_dropping_tile_pos_y;
				m_n_score += 7;
			} else
				Lay_Tile();

			if(!m_b_game_over) {
#if defined(_WIN32) || defined(_WIN64)
				char p_s_title[256];
				sprintf(p_s_title, "Sparse Tetris: %d points", m_n_score);
				SetWindowText(m_h_wnd, p_s_title);
#endif // _WIN32 || _WIN64
			}
		}
	}

	/**
	 *	@brief "bakes" the tile into playing field once it collided,
	 *		and detects and processes full rows, if any
	 */
	void Lay_Tile()
	{
		uint16_t n_tile = n_Tile(m_n_dropping_tile_type, m_n_dropping_tile_angle);
		for(int j = 0; j < 4; ++ j) {
			for(int i = 0; i < 4; ++ i) {
				if(((n_tile >> (i + 4 * j)) & 1) != 0) {
					int n_col = i + m_n_dropping_tile_pos_x;
					int n_row = j + m_n_dropping_tile_pos_y;
					if(n_col >= 0 && n_col < m_n_field_width &&
					   n_row >= 0 && n_row < m_n_field_height)
						m_field[n_row * m_n_field_width + n_col] = true;
				}
			}
		}
		// "bake" the tile into the field

		if(m_n_dropping_tile_pos_y <= 0) {
			m_b_game_over = true;
#if defined(_WIN32) || defined(_WIN64)
			KillTimer(m_h_wnd, 0);
#ifdef __TIMER_POWERED_AI
			if(m_b_AI_game)
				KillTimer(m_h_wnd, 1);
#endif // __TIMER_POWERED_AI
			char p_s_title[256];
			sprintf(p_s_title, "Sparse Tetris: %d points | Game Over", m_n_score);
			SetWindowText(m_h_wnd, p_s_title);
			InvalidateRect(m_h_wnd, 0, false); // force repaint
			SetTimer(m_h_wnd, 2, 10000, 0); // set a restart game timer
#endif // _WIN32 || _WIN64

			{
				printf("\ngame over with %d points\n", m_n_score);
				printf("+");
				for(int i = 0; i < m_n_field_width; ++ i)
					printf("-");
				printf("+\n");
				for(int j = 0; j < m_n_field_height; ++ j) {
					printf("|");
					for(int i = 0; i < m_n_field_width; ++ i)
						printf("%c", (m_field[i + j * m_n_field_width])? '#' : ' ');
					printf("|\n");
				}
				printf("+");
				for(int i = 0; i < m_n_field_width; ++ i)
					printf("-");
				printf("+\n");
			}
			// print game state ...

			return;
		}
		// detect "game over"

		for(int j = 0; j < m_n_field_height; ++ j) {
			bool b_straight_line = true;
			for(int i = 0; i < m_n_field_width; ++ i) {
				if(!m_field[i + m_n_field_width * j]) {
					b_straight_line = false;
					break;
				}
			}
			// detect straight line

			if(b_straight_line) {
				for(int k = j; k > 0; -- k) {
					for(int i = 0; i < m_n_field_width; ++ i)
						m_field[i + m_n_field_width * k] = m_field[i + m_n_field_width * (k - 1)];
				}
				// shift lines down

				for(int i = 0; i < m_n_field_width; ++ i)
					m_field[i + m_n_field_width * 0] = false;
				// at the top-most line, just clear it (nowhere to shift from)

				m_n_score += 1000;
			}
		}
		// detect line clearing

		m_n_dropping_tile_type = rand();
		m_n_dropping_tile_pos_x = m_n_field_width / 2 - 2;
		m_n_dropping_tile_pos_y = -4;
		m_n_dropping_tile_angle = 0;
		// make a new tile
	}

	/**
	 *	@brief fills the current active into the block matrix
	 *	@param[out] bm is the output block matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Fill_Tile(CUberBlockMatrix &bm) // throw(std::bad_alloc)
	{
		if(m_b_game_over)
			return;

		uint16_t n_tile = n_Tile(m_n_dropping_tile_type, m_n_dropping_tile_angle);
		if(m_b_backwards_fill) {
			if(m_b_swap_loops) {
				for(int i = 3; i >= 0; -- i) {
					for(int j = 3; j >= 0; -- j) {
						if(((n_tile >> (i + 4 * j)) & 1) != 0) {
							int n_col = i + m_n_dropping_tile_pos_x;
							int n_row = j + m_n_dropping_tile_pos_y;
							if(n_col >= 0 && n_col < m_n_field_width &&
							   n_row >= 0 && n_row < m_n_field_height) {
								if(m_b_vertical_fill)
									bm.Append_Block(m_block, n_col * m_block.rows(), n_row * m_block.cols());
								else
									bm.Append_Block(m_block, n_row * m_block.rows(), n_col * m_block.cols());
							}
						}
					}
				}
			} else {
				for(int j = 3; j >= 0; -- j) {
					for(int i = 3; i >= 0; -- i) {
						if(((n_tile >> (i + 4 * j)) & 1) != 0) {
							int n_col = i + m_n_dropping_tile_pos_x;
							int n_row = j + m_n_dropping_tile_pos_y;
							if(n_col >= 0 && n_col < m_n_field_width &&
							   n_row >= 0 && n_row < m_n_field_height) {
								if(m_b_vertical_fill)
									bm.Append_Block(m_block, n_col * m_block.rows(), n_row * m_block.cols());
								else
									bm.Append_Block(m_block, n_row * m_block.rows(), n_col * m_block.cols());
							}
						}
					}
				}
			}
		} else {
			if(m_b_swap_loops) {
				for(int i = 0; i < 4; ++ i) {
					for(int j = 0; j < 4; ++ j) {
						if(((n_tile >> (i + 4 * j)) & 1) != 0) {
							int n_col = i + m_n_dropping_tile_pos_x;
							int n_row = j + m_n_dropping_tile_pos_y;
							if(n_col >= 0 && n_col < m_n_field_width &&
							   n_row >= 0 && n_row < m_n_field_height) {
								if(m_b_vertical_fill)
									bm.Append_Block(m_block, n_col * m_block.rows(), n_row * m_block.cols());
								else
									bm.Append_Block(m_block, n_row * m_block.rows(), n_col * m_block.cols());
							}
						}
					}
				}
			} else {
				for(int j = 0; j < 4; ++ j) {
					for(int i = 0; i < 4; ++ i) {
						if(((n_tile >> (i + 4 * j)) & 1) != 0) {
							int n_col = i + m_n_dropping_tile_pos_x;
							int n_row = j + m_n_dropping_tile_pos_y;
							if(n_col >= 0 && n_col < m_n_field_width &&
							   n_row >= 0 && n_row < m_n_field_height) {
								if(m_b_vertical_fill)
									bm.Append_Block(m_block, n_col * m_block.rows(), n_row * m_block.cols());
								else
									bm.Append_Block(m_block, n_row * m_block.rows(), n_col * m_block.cols());
							}
						}
					}
				}
			}
		}
	}

	/**
	 *	@brief fills the playing field into the block matrix
	 *	@param[out] bm is the output block matrix
	 *	@note This function throws std::bad_alloc.
	 */
	void Fill_Field(CUberBlockMatrix &bm) // throw(std::bad_alloc)
	{
		if(m_b_backwards_fill) {
			if(m_b_swap_loops) {
				for(int i = m_n_field_width - 1; i >= 0; -- i) {
					for(int j = m_n_field_height - 1; j >= 0; -- j) {
						if(m_field[i + m_n_field_width * j]) {
							if(m_b_vertical_fill)
								bm.Append_Block(m_block, i * m_block.rows(), j * m_block.cols());
							else
								bm.Append_Block(m_block, j * m_block.rows(), i * m_block.cols());
						}
					}
				}
			} else {
				for(int j = m_n_field_height - 1; j >= 0; -- j) {
					for(int i = m_n_field_width - 1; i >= 0; -- i) {
						if(m_field[i + m_n_field_width * j]) {
							if(m_b_vertical_fill)
								bm.Append_Block(m_block, i * m_block.rows(), j * m_block.cols());
							else
								bm.Append_Block(m_block, j * m_block.rows(), i * m_block.cols());
						}
					}
				}
			}
		} else {
			if(m_b_swap_loops) {
				for(int j = 0; j < m_n_field_height; ++ j) {
					for(int i = 0; i < m_n_field_width; ++ i) {
						if(m_field[i + m_n_field_width * j]) {
							if(m_b_vertical_fill)
								bm.Append_Block(m_block, i * m_block.rows(), j * m_block.cols());
							else
								bm.Append_Block(m_block, j * m_block.rows(), i * m_block.cols());
						}
					}
				}
			} else {
				for(int i = 0; i < m_n_field_width; ++ i) {
					for(int j = 0; j < m_n_field_height; ++ j) {
						if(m_field[i + m_n_field_width * j]) {
							if(m_b_vertical_fill)
								bm.Append_Block(m_block, i * m_block.rows(), j * m_block.cols());
							else
								bm.Append_Block(m_block, j * m_block.rows(), i * m_block.cols());
						}
					}
				}
			}
		}
	}

#if defined(_WIN32) || defined(_WIN64)
	/**
	 *	@brief windows message loop callback
	 *
	 *	@param[in] h_wnd is handle to the window that receives the message
	 *	@param[in] n_msg is name of the mesage
	 *	@param[in] n_w_param is the first message parameter
	 *	@param[in] n_l_param is the second message parameter
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	inline LRESULT CALLBACK WndProc(HWND h_wnd, UINT n_msg, WPARAM n_w_param, LPARAM n_l_param) // throw(std::bad_alloc)
	{
		switch(n_msg) {
		case WM_TIMER:
			if(n_w_param == 2) {
				Restart(); // restart game
				KillTimer(m_h_wnd, 2); // once
#ifdef __TIMER_POWERED_AI
				if(m_b_AI_game)
					SetTimer(m_h_wnd, 1, 1, NULL);
#endif // __TIMER_POWERED_AI
				SetTimer(m_h_wnd, 0, 500, NULL);
				// set timer for animations
			}
#ifdef __TIMER_POWERED_AI
			else if(n_w_param == 1)
				AI_NextStep();
#endif // __TIMER_POWERED_AI
			else {
				if(b_TileColided(0, 1))
					Lay_Tile();
				else {
					m_n_dropping_tile_pos_y ++;
					m_n_score += 5;
				}
				// animate the tile movement

				if(!m_b_game_over) {
					char p_s_title[256];
					sprintf(p_s_title, "Sparse Tetris: %d points", m_n_score);
					SetWindowText(m_h_wnd, p_s_title);
				}
			}

			InvalidateRect(m_h_wnd, 0, false); // force repaint
			return 0;

		case WM_KEYDOWN:
			if(n_w_param == VK_ESCAPE) {
				SendMessage(m_h_wnd, WM_CLOSE, 0, 0);
				return 0;
			}
			if(!m_b_AI_game && !m_b_game_over) {
				bool b_reset_timer = true;
				switch(n_w_param) {
				case VK_LEFT:
				case VK_RIGHT:
					if(!b_TileColided(((n_w_param == VK_LEFT)? -1 : 1), 0))
						m_n_dropping_tile_pos_x += ((n_w_param == VK_LEFT)? -1 : 1);
					break;
				case VK_UP:
					if(!b_TileColided(0, 0, 1))
						m_n_dropping_tile_angle = (m_n_dropping_tile_angle + 1) % 4;
					break;
				case VK_DOWN:
					if(!b_TileColided(0, 1)) {
						++ m_n_dropping_tile_pos_y;
						m_n_score += 7;
					} else
						Lay_Tile();

					if(!m_b_game_over) {
						char p_s_title[256];
						sprintf(p_s_title, "Sparse Tetris: %d points", m_n_score);
						SetWindowText(m_h_wnd, p_s_title);
					}
					break;
				default:
					b_reset_timer = false;
				};
				if(b_reset_timer) {
					KillTimer(m_h_wnd, 0);
					SetTimer(m_h_wnd, 0, 500, NULL);
					InvalidateRect(m_h_wnd, 0, false); // force repaint
				}
			}
			break;

		case WM_PAINT:
			{
				CUberBlockMatrix bm;
				if(m_b_fill_tile_first)
					Fill_Tile(bm);
				Fill_Field(bm);
				if(!m_b_fill_tile_first)
					Fill_Tile(bm);
				// fills the über block matrix

				m_p_bitmap = bm.p_Rasterize(m_p_bitmap, m_n_scalar_size);
				// copy the pointer

				PAINTSTRUCT t_ps;
				HDC h_dc = BeginPaint(h_wnd, &t_ps);

				int n_bw = m_p_bitmap->n_width;
				int n_bh = m_p_bitmap->n_height;
				// get bitmap dimensions

				RECT t_rect;
				GetClientRect(h_wnd, &t_rect);
				int n_ww = t_rect.right - t_rect.left;
				int n_wh = t_rect.bottom - t_rect.top;
				// get window dimensions

				BITMAPINFO t_bi;
				t_bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
				t_bi.bmiHeader.biWidth = n_bw;
				t_bi.bmiHeader.biHeight = -n_bh; // flip the image upside down
				t_bi.bmiHeader.biPlanes = 1;
				t_bi.bmiHeader.biBitCount = 8 * sizeof(uint32_t);
				t_bi.bmiHeader.biCompression = BI_RGB;
				t_bi.bmiHeader.biSizeImage = n_bw * n_bh * sizeof(uint32_t);
				t_bi.bmiHeader.biXPelsPerMeter = 0;
				t_bi.bmiHeader.biYPelsPerMeter = 0;
				t_bi.bmiHeader.biClrUsed = 0;
				t_bi.bmiHeader.biClrImportant = 0;
				// create bitmap info

				/*SetStretchBltMode(h_dc, HALFTONE);
				SetBrushOrgEx(h_dc, 0, 0, NULL);*/
				// high quality stretch-blt

				if(n_ww > n_bw || n_wh > n_bh) {
					RECT t_rect;
					t_rect.left = n_bw;
					t_rect.right = n_ww;
					t_rect.top = 0;
					t_rect.bottom = n_wh;
					if(n_ww > n_bw)
						FillRect(h_dc, &t_rect, WHITE_BRUSH);
					t_rect.top = n_bh;
					t_rect.left = 0;
					t_rect.right = n_bw;
					if(n_wh > n_bh)
						FillRect(h_dc, &t_rect, WHITE_BRUSH);
				}
				// clear background next to and below the bitmap

				StretchDIBits(h_dc, 0, 0, n_bw, n_bh, 0, 0, n_bw, n_bh,
					m_p_bitmap->p_buffer, &t_bi, DIB_RGB_COLORS, SRCCOPY);
				// paint bitmap (keep it in the lower-left corner, no stretch)

				EndPaint(h_wnd, &t_ps);
			}
			return 0;

		case WM_DESTROY:
			PostQuitMessage(0);
			return 0;
		}

		return DefWindowProc(h_wnd, n_msg, n_w_param, n_l_param);
	}

	/**
	 *	@brief windows message loop callback (static call redirection)
	 *
	 *	@param[in] h_wnd is handle to the window that receives the message
	 *	@param[in] n_msg is name of the mesage
	 *	@param[in] n_w_param is the first message parameter
	 *	@param[in] n_l_param is the second message parameter
	 *
	 *	@note This function throws std::bad_alloc.
	 */
	static LRESULT CALLBACK _WndProc(HWND h_wnd, UINT n_msg, WPARAM n_w_param, LPARAM n_l_param) // throw(std::bad_alloc)
	{
		_ASSERTE(m_p_instance);
		return m_p_instance->WndProc(h_wnd, n_msg, n_w_param, n_l_param);
	}
#endif // _WIN32 || _WIN64
};

#if defined(_WIN32) || defined(_WIN64)
CTetrisGame *CTetrisGame::m_p_instance = 0; /**< @brief instance of the tetris game (it's a singleton) */
#endif // _WIN32 || _WIN64

/**
 *	@brief main entry point for tetris game
 *
 *	@param[in] n_arg_num is number of arguments
 *	@param[in] p_arg_list is list of arguments
 *
 *	@return Returns 0 on success, -1 on failure.
 */
int tetris_main(int n_arg_num, const char **p_arg_list)
{
	int n_field_width = 20;
	int n_field_height = 20;
	int n_block_width = 3;
	int n_block_height = 3;
	bool b_AI = false;
	bool b_vertical_fill = false;
	bool b_backwards_fill = false;
	bool b_fill_tile_first = false;
	bool b_row_major_fill = false;
	int n_scalar_size = 5;

	for(int i = 1; i < n_arg_num; ++ i) {
		if(!strcmp(p_arg_list[i], "--help") || !strcmp(p_arg_list[i], "-h")) {
			printf("no help for you, bwahahaha!\n");//PrintHelp();
			return 0;
		} else if(!strcmp(p_arg_list[i], "--AI-game") || !strcmp(p_arg_list[i], "-ai")) {
			b_AI = true;
		} else if(!strcmp(p_arg_list[i], "--vertical-fill") || !strcmp(p_arg_list[i], "-vf")) {
			b_vertical_fill = true;
		} else if(!strcmp(p_arg_list[i], "--backwards-fill") || !strcmp(p_arg_list[i], "-bf")) {
			b_backwards_fill = true;
		} else if(!strcmp(p_arg_list[i], "--fill-tile-first") || !strcmp(p_arg_list[i], "-ftf")) {
			b_fill_tile_first = true;
		} else if(!strcmp(p_arg_list[i], "--row-major-fill") || !strcmp(p_arg_list[i], "-rmf")) {
			b_row_major_fill = true;
		} else if(i + 1 == n_arg_num) {
			fprintf(stderr, "error: argument \'%s\': missing value or an unknown argument\n", p_arg_list[i]);
			return -1;
		} else if(!strcmp(p_arg_list[i], "--game-width") || !strcmp(p_arg_list[i], "-gw")) {
			n_field_width = atol(p_arg_list[++ i]);
		} else if(!strcmp(p_arg_list[i], "--game-height") || !strcmp(p_arg_list[i], "-gh")) {
			n_field_height = atol(p_arg_list[++ i]);
		} else if(!strcmp(p_arg_list[i], "--block-width") || !strcmp(p_arg_list[i], "-bw")) {
			n_block_width = atol(p_arg_list[++ i]);
		} else if(!strcmp(p_arg_list[i], "--block-height") || !strcmp(p_arg_list[i], "-bh")) {
			n_block_height = atol(p_arg_list[++ i]);
		} else if(!strcmp(p_arg_list[i], "--scalar-size") || !strcmp(p_arg_list[i], "-ss")) {
			n_scalar_size = atol(p_arg_list[++ i]);
		} else {
			fprintf(stderr, "error: argument \'%s\': an unknown argument\n", p_arg_list[i]);
			return -1;
		}
	}
	// "parse" cmdline

	printf("running with the following configuration:\n");
	printf("n_field_width = %d\n", n_field_width);
	printf("n_field_height = %d\n", n_field_height);
	printf("n_block_width = %d\n", n_block_width);
	printf("n_block_height = %d\n", n_block_height);
	printf("b_AI = %s\n", (b_AI)? "true" : "false");
	printf("b_vertical_fill = %s\n", (b_vertical_fill)? "true" : "false");
	printf("b_backwards_fill = %s\n", (b_backwards_fill)? "true" : "false");
	printf("b_fill_tile_first = %s\n", (b_fill_tile_first)? "true" : "false");
	printf("b_row_major_fill = %s\n", (b_row_major_fill)? "true" : "false");
	printf("n_scalar_size = %d\n", n_scalar_size);

	CTetrisGame g(b_vertical_fill, b_backwards_fill, b_fill_tile_first, b_row_major_fill,
		n_field_width, n_field_height, n_block_width, n_block_height, n_scalar_size);
	g.Run(b_AI);
	// play tetris

	return 0;
}

#endif // !__TETRIS_INCLUDED
