/*
								+----------------------------------+
								|                                  |
								| ***  8bit to unicode tables  *** |
								|                                  |
								|   Copyright © -tHE SWINe- 2011   |
								|                                  |
								|           UniTables.h            |
								|                                  |
								+----------------------------------+
*/

#pragma once
#ifndef __8BIT_CHARSETS_TO_UNICODE_CONVERSION_TABLES_INCLUDED
#define __8BIT_CHARSETS_TO_UNICODE_CONVERSION_TABLES_INCLUDED

/**
 *	@file UniTables.h
 *	@brief 8bit charsets to unicode mapping tables
 *	@date 2011
 *	@author -tHE SWINe-
 *
 *	@date 2012-06-19
 *
 *	Added \#pragma once.
 *
 *	@date 2012-10-18
 *
 *	Changed m_charset_map to m_p_charset_map to avoid leaking memory
 *	(std::map seems to allocate something, even if empty).
 */

#include <string>
#include <map>
#include <memory>

class CUnicodeMappingDirectory;

#include "UniConv.h"
#include "Dir.h"

/**
 *	@brief a simple implementation of charsets conversion tables directory
 */
class CUnicodeMappingDirectory {
protected:
	typedef std::pair<const CUnicodeMapping::TCharacterName*, size_t> TMappingTable;
	typedef std::map<std::string, TMappingTable> TUniMappingMap;
	static TUniMappingMap *m_p_charset_map;
	static std::auto_ptr<TUniMappingMap> m_charset_map_ptr;

	class CMappingDirectoryBuilder {
	protected:
		struct TTableInfo {
			CUnicodeMapping::CUnicodeMappingTable *first;
			std::string second, third;

			inline TTableInfo()
			{}

			inline TTableInfo(CUnicodeMapping::CUnicodeMappingTable *_first,
				const std::string &_second, const std::string &_third)
				:first(_first), second(_second), third(_third)
			{}
		};

		std::vector<TTableInfo> m_table_list;
		bool m_b_iso;

		mutable std::string m_s_safe_temp;

	public:
		CMappingDirectoryBuilder();
		~CMappingDirectoryBuilder();
		bool BeginVendorDirectory();
		bool Dump(FILE *p_fw = stdout);
		void operator ()(const TFileInfo &r_t_file); // throws(std::bad_alloc)

	protected:
		const char *p_s_DocCommentStr(const char *p_s_str) const; // throws(std::bad_alloc)
		const char *p_s_SafeStr(const char *p_s_str) const; // throws(std::bad_alloc)
		void Dump_UnicodeMappingTable(const CUnicodeMapping::CUnicodeMappingTable &r_table,
			FILE *p_fw, const char *p_s_charset_name,
			const char *p_s_variable_name_prefix) const; // throws(std::bad_alloc)
		static inline void MakeIdent(char &r_n_char);
		static inline void ToLower(char &r_n_char);
		static inline void DeleteTable(TTableInfo &r_item);
	};

public:
	/**
	 *	@brief gets a raw unicode mapping table for a specified chatset
	 *
	 *	@param[in] p_s_charset is charset name (eg. "iso-8859-1")
	 *	@param[out] r_n_table_size is filled with number of mapping table entries
	 *
	 *	@return Returns pointer to raw unicode mapping table for a specified chatset
	 *		on success, or 0 on failure.
	 */
	static const CUnicodeMapping::TCharacterName *p_GetUnicodeMapping(
		const char *p_s_charset, size_t &r_n_table_size);

	/**
	 *	@brief generates source code for the UniTables.inl file
	 *	@param[in] p_s_encodings_path is path to the directory with encodings
	 *	@return Returns true on success, false on failure.
	 *	@note The directory with encodings is suppoed to contain "iso" and "vendor"
	 *		subdirectories. Get files from http://www.unicode.org/Public/MAPPINGS/.
	 */
	static bool Build_TablesFile(const char *p_s_encodings_path = "./charsets");
};

inline CUnicodeMapping::CUnicodeMapping(const char *p_s_charset_name,
	bool &r_b_charset_found, bool b_avoid_accents)
	:m_n_subst_char(-1), m_n_inverse_map_size(0)
{
	size_t n_table_size;
	const TCharacterName *p_table = CUnicodeMappingDirectory::p_GetUnicodeMapping(
		p_s_charset_name, n_table_size);
	if(!(r_b_charset_found = (p_table != 0)) ||
	   !FromTable(p_table, n_table_size, b_avoid_accents))
		m_n_inverse_map_size = 0; // to mark error
}

#endif // !__8BIT_CHARSETS_TO_UNICODE_CONVERSION_TABLES_INCLUDED
