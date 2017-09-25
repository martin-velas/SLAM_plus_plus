/*
								+----------------------------------+
								|                                  |
								|   ***  Unicode block list  ***   |
								|                                  |
								|   Copyright © -tHE SWINe- 2011   |
								|                                  |
								|           UniBlocks.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __UNICODE_BLOCK_NAMES_INCLUDED
#define __UNICODE_BLOCK_NAMES_INCLUDED

/**
 *	@file UniBlocks.h
 *	@date 2011
 *	@author -tHE SWINe-
 *	@brief Unicode blocks and their allocation throughout different unicode versions.
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 */

/**
 *	@brief Unicode blocks and their allocation throughout different unicode versions
 */
class CUnicodeBlocks {
public:
	/**
	 *	@brief inclusive range of character codes
	 */
	struct TCodeRange {
		int n_first_code; /**< @brief the first code point */
		int n_last_code; /**< @brief the last code point (inclusive) */
	};

	/**
	 *	@brief named unicode block
	 */
	struct TUnicodeLogBlock {
		TCodeRange t_range; /**< @brief inclusive range of character codes in this block */
		const char *p_s_name; /**< @brief block name (eg. "Basic Latin") */
	};

	/**
	 *	@brief versioned unicode allocation table
	 */
	struct TUnicodeAllocation {
		const TCodeRange *p_code_range; /**< @brief a list of allocated code ranges (these overlap with unicode blocks, the only difference between the different unicode versions is amount of characters allocated in each block) */
		size_t n_range_num; /**< @brief number of allocated code ranges */
		const char *p_s_unicode_version; /**< @brief unicode version string (eg. "1.1.5") */
	};

protected:
	static const TUnicodeLogBlock p_unicode_blocks[]; /**< @brief list of all known named unicode blocks */
	static const size_t n_unicode_log_block_num; /**< @brief number of named unicode blocks */
	static const TCodeRange p_global_range_list[]; /**< @brief list of character ranges, containing data for all the unicode versions */
	static const TUnicodeAllocation p_allocation_table[]; /**< @brief character allocation table for all unicode versions */
	static const size_t n_allocation_table_size; /**< @brief number of unicode versions */

public:
	/**
	 *	@brief translates unicode versions string to unicode version id
	 *	@param[in] p_s_unicode_version is unicode version string (eg. "1.1.5")
	 *	@return Returns unicode version id, or -1 if the requested version is not supported.
	 */
	static size_t n_Translate_UnicodeVersion(const char *p_s_unicode_version);

	/**
	 *	@brief translates unicode block names to ranges of codepoints
	 *
	 *	@param[out] r_range_list is list of character ranges
	 *	@param[out] r_n_char_num is total number of characters
	 *	@param[in] n_unicode_block_num is number of unicode blocks
	 *	@param[in] p_unicode_block_list is list of unicode block names
	 *	@param[in] n_unicode_version_index is unicode version id
	 *		(see n_Translate_UnicodeVersion() function documentation)
	 *	@param[in] b_errors_to_stderr is error display flag (if set, all the
	 *		bad block names are printed to stderr)
	 *
	 *	@return Returns true on success, false on failure.
	 *
	 *	@note This adds character ranges to the list, so r_n_char_num must be zero
	 *		and r_range_list must be empty at the time of the first call to this function.
	 */
	static bool Codepoints_From_UnicodeNames(std::vector<std::pair<int, int> > &r_range_list,
		size_t &r_n_char_num, size_t n_unicode_block_num, const char **p_unicode_block_list,
		size_t n_unicode_version_id, bool b_errors_to_stderr = false);

protected:
	static inline bool b_Smaller_FirstCode(const TCodeRange &a, const TCodeRange &b);
};

#endif // !__UNICODE_BLOCK_NAMES_INCLUDED
