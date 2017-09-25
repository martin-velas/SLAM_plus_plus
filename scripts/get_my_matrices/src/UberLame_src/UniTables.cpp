/*
								+----------------------------------+
								|                                  |
								| ***  8bit to unicode tables  *** |
								|                                  |
								|   Copyright © -tHE SWINe- 2011   |
								|                                  |
								|          UniTables.cpp           |
								|                                  |
								+----------------------------------+
*/

/**
 *	@file UniTables.cpp
 *	@brief 8bit charsets to unicode mapping tables
 *	@date 2011
 *	@author -tHE SWINe-
 *
 *	@date 2012-06-19
 *
 *	Added code to escape # by \ in documentation comments in generated UniTables.inl.
 */

#include "NewFix.h"
#include "CallStack.h"
#include <stdio.h>
#include <algorithm>
#include "UniTables.h"
#include "UniTables.inl"

using namespace __8bit_to_unicode_mapping_tables;

#if defined(_WIN32) || defined(_WIN64)
#define strncasecmp(a,b,n) _strnicmp((a), (b), (n))
#define strcasecmp(a,b) _stricmp((a), (b))
#endif //_WIN32 || _WIN64

/*
 *								=== CUnicodeMappingDirectory ===
 */

CUnicodeMappingDirectory::TUniMappingMap *CUnicodeMappingDirectory::m_p_charset_map;
std::auto_ptr<CUnicodeMappingDirectory::TUniMappingMap>
	CUnicodeMappingDirectory::m_charset_map_ptr(0);

const CUnicodeMapping::TCharacterName *CUnicodeMappingDirectory::p_GetUnicodeMapping(
	const char *p_s_charset, size_t &r_n_table_size)
{
	try {
		if(!m_p_charset_map) {
			m_p_charset_map = new TUniMappingMap();
			m_charset_map_ptr = std::auto_ptr<TUniMappingMap>(m_p_charset_map);
		}
		if(m_p_charset_map->size() != n_mapping_num) {
			m_p_charset_map->clear(); // !!
			for(size_t i = 0; i < n_mapping_num; ++ i) {
				(*m_p_charset_map)[p_mappings_table[i].p_s_name] = std::make_pair(p_mappings_table[i].p_table,
					p_mappings_table[i].n_table_size);
			}
			_ASSERTE(m_p_charset_map->size() == n_mapping_num);
		}
		// make sure the map is filled

		std::string s_temp;
		if(strlen(p_s_charset) > 8 && p_s_charset[7] == '-' &&
		   !strncasecmp(p_s_charset, "windows", 7)) {
			s_temp = "cp-";
			s_temp += p_s_charset + 8;
			p_s_charset = s_temp.c_str();
		}
		// "windows-*" is stored as "cp-*"

		TUniMappingMap::const_iterator p_mapping_iter = m_p_charset_map->find(p_s_charset);
		if(p_mapping_iter == m_p_charset_map->end())
			return 0;
		// look up the charset

		r_n_table_size = (*p_mapping_iter).second.second; // !!
		return (*p_mapping_iter).second.first;
	} catch(std::bad_alloc&) {
		return 0;
	}
}

bool CUnicodeMappingDirectory::Build_TablesFile(const char *p_s_encodings_path)
{
	std::string s_path_iso, s_path_vendors;
	if(!stl_ut::AssignCStr(s_path_iso, p_s_encodings_path) ||
	   !stl_ut::AppendCStr(s_path_iso, "\\iso") ||
	   !stl_ut::AssignCStr(s_path_vendors, p_s_encodings_path) ||
	   !stl_ut::AppendCStr(s_path_vendors, "\\vendor"))
		return false;

	CMappingDirectoryBuilder rep_adder;

	try {
		if(!CDirTraversal::Traverse(s_path_iso.c_str(), rep_adder, false) ||
		   !rep_adder.BeginVendorDirectory() ||
		   !CDirTraversal::Traverse(s_path_vendors.c_str(), rep_adder))
			return false;
	} catch(std::bad_alloc&) {
		return false;
	}

	return rep_adder.Dump();
}

/*
 *								=== ~CUnicodeMappingDirectory ===
 */

/*
 *								=== CUnicodeMappingDirectory::CMappingDirectoryBuilder ===
 */

CUnicodeMappingDirectory::CMappingDirectoryBuilder::CMappingDirectoryBuilder()
	:m_b_iso(true)
{}

CUnicodeMappingDirectory::CMappingDirectoryBuilder::~CMappingDirectoryBuilder()
{
	std::for_each(m_table_list.begin(), m_table_list.end(), DeleteTable);
}

bool CUnicodeMappingDirectory::CMappingDirectoryBuilder::BeginVendorDirectory()
{
	m_b_iso = false;
	return true;
}

bool CUnicodeMappingDirectory::CMappingDirectoryBuilder::Dump(FILE *p_fw)
{
	fprintf(p_fw, "/*\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t+----------------------------------+\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|                                  |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t| ***  8bit to unicode tables  *** |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|                                  |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|   Copyright © -tHE SWINe- 2011   |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|                                  |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|          UniTables.inl           |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t|                                  |\n");
	fprintf(p_fw, "\t\t\t\t\t\t\t\t+----------------------------------+\n");
	fprintf(p_fw, "*/\n\n");
	fprintf(p_fw, "/**\n"
				  " *\t@file UniTables.inl\n"
				  " *\t@brief 8bit charsets to unicode mapping tables\n"
				  " *\t@date 2011\n"
				  " *\t@author -tHE SWINe-\n"
				  " */\n\n");

	fprintf(p_fw, "namespace __8bit_to_unicode_mapping_tables {\n");

	for(size_t i = 0, n = m_table_list.size(); i < n; ++ i) {
		CUnicodeMapping::CUnicodeMappingTable *p_cum = m_table_list[i].first;
		std::string &r_s_charset = m_table_list[i].second;
		std::string &r_s_ident = m_table_list[i].third;

		try {
			Dump_UnicodeMappingTable(*p_cum, p_fw, r_s_charset.c_str(), r_s_ident.c_str());
		} catch(std::bad_alloc&) {
			return false;
		}
		// dump
	}

	fprintf(p_fw, "\nstatic const struct {\n"
				  "\tconst char *p_s_name;\n"
				  "\tconst CUnicodeMapping::TCharacterName *p_table;\n"
				  "\tsize_t n_table_size;\n"
				  "} p_mappings_table[] = {\n");
	for(size_t j = 0, m = m_table_list.size(); j < m; ++ j) {
		CUnicodeMapping::CUnicodeMappingTable *p_cum = m_table_list[j].first;
		std::string &r_s_charset = m_table_list[j].second;
		std::string &r_s_ident = m_table_list[j].third;

		fprintf(p_fw, "\t{\"%s\", p_%s_table, n_%s_table_size}%s\n",
			r_s_charset.c_str(), r_s_ident.c_str(), r_s_ident.c_str(), (j + 1 < m)? "," : "");
	}
	fprintf(p_fw, "};\n");
	fprintf(p_fw, "const size_t n_mapping_num = sizeof(p_mappings_table) / sizeof(p_mappings_table[0]);\n");

	fprintf(p_fw, "\n}; // ~__8bit_to_unicode_mapping_tables\n");

	return !ferror(p_fw);
}

void CUnicodeMappingDirectory::CMappingDirectoryBuilder::operator ()(
	const TFileInfo &r_t_file) // throws(std::bad_alloc)
{
	if(r_t_file.b_directory)
		return;
	if(strcasecmp(r_t_file.p_s_Extension(), "txt"))
		return;
	// only interested in txt file

	CUnicodeMapping::CUnicodeMappingTable *p_cum = new CUnicodeMapping::CUnicodeMappingTable();
	if(!p_cum->Load(r_t_file.p_s_Path())) {
		fprintf(stderr, "warning: file \'%s\' failed to load ...\n", r_t_file.p_s_Path());
		delete p_cum;
		return;
	}
	// try to load the table

	const char *p_s_encoding = r_t_file.p_s_FileName();
	std::string s_charset = std::string((m_b_iso)? "iso-" : "") + p_s_encoding;
	s_charset.erase(s_charset.rfind('.'));
	std::for_each(s_charset.begin(), s_charset.end(), ToLower);
	if(!m_b_iso) {
		if(s_charset.length() > 2 && s_charset[0] == 'c' && s_charset[1] == 'p')
			s_charset.insert(2, "-");
	}
	// generate charset name

	std::string s_ident(s_charset);
	if(s_ident.empty() || isdigit(uint8_t(s_ident[0])))
		s_ident.insert(0, "_");
	std::for_each(s_ident.begin(), s_ident.end(), MakeIdent);
	// make "c" ident out of charset name

	m_table_list.push_back(TTableInfo(p_cum, s_charset, s_ident));
	// add it to the list
}

const char *CUnicodeMappingDirectory::CMappingDirectoryBuilder::p_s_DocCommentStr(
	const char *p_s_str) const // throws(std::bad_alloc)
{
	m_s_safe_temp = p_s_str;
	if(!m_s_safe_temp.empty() && m_s_safe_temp[0] == '#') {
		m_s_safe_temp[0] = ' ';
		m_s_safe_temp.insert(1, "*");
	}
	{
		size_t n_pos = 0;
		while((n_pos = m_s_safe_temp.find("\n#", n_pos)) != std::string::npos) {
			m_s_safe_temp[n_pos + 1] = ' ';
			m_s_safe_temp.insert(n_pos + 2, "*");
			++ n_pos;
		}
	}
	{
		size_t n_pos = 0;
		while((n_pos = m_s_safe_temp.find("#", n_pos)) != std::string::npos) {
			m_s_safe_temp.insert(n_pos, "\\");
			n_pos += 2;
		}
	}
	return m_s_safe_temp.c_str();
}

const char *CUnicodeMappingDirectory::CMappingDirectoryBuilder::p_s_SafeStr(
	const char *p_s_str) const // throws(std::bad_alloc)
{
	m_s_safe_temp = p_s_str;
	size_t n_pos = 0;
	while((n_pos = m_s_safe_temp.find('\"', n_pos)) != std::string::npos) {
		if(n_pos == 0 || m_s_safe_temp[n_pos - 1] != '\\') {
			m_s_safe_temp.insert(n_pos, "\\");
			++ n_pos;
		}
		++ n_pos;
	}
	n_pos = 0;
	while((n_pos = m_s_safe_temp.find('\\', n_pos)) != std::string::npos) {
		if((n_pos == 0 || m_s_safe_temp[n_pos - 1] != '\\') &&
		   (n_pos + 1 == m_s_safe_temp.length() || (m_s_safe_temp[n_pos + 1] != '\\' &&
		   m_s_safe_temp[n_pos + 1] != '\"'))) {
			m_s_safe_temp.insert(n_pos, "\\");
			++ n_pos;
		}
		++ n_pos;
	}
	return m_s_safe_temp.c_str();
}

void CUnicodeMappingDirectory::CMappingDirectoryBuilder::Dump_UnicodeMappingTable(
	const CUnicodeMapping::CUnicodeMappingTable &r_table,
	FILE *p_fw, const char *p_s_charset_name,
	const char *p_s_variable_name_prefix) const // throws(std::bad_alloc)
{
	fprintf(p_fw, "\n/**\n *\t@brief %s to unicode mapping table\n *\n", p_s_charset_name);
	fprintf(p_fw, "%s", p_s_DocCommentStr(r_table.s_Notice().c_str()));
	fprintf(p_fw, " */\n");
	fprintf(p_fw, "const CUnicodeMapping::TCharacterName p_%s_table[] = {\n", p_s_variable_name_prefix);
	const CUnicodeMapping::TCharacterName *m_p_map = r_table.p_Table();
	for(size_t i = 0, n = r_table.n_Table_Size(); i < n; ++ i) {
		fprintf(p_fw, "\t{0x%02x, 0x%04x, \"%s\"}%s\n", m_p_map[i].n_char8, m_p_map[i].n_unicode,
			p_s_SafeStr((m_p_map[i].p_s_name)? m_p_map[i].p_s_name : ""), (i + 1 < n)? "," : "");
	}
	fprintf(p_fw, "};\n");
	fprintf(p_fw, "const size_t n_%s_table_size = sizeof(p_%s_table) / sizeof(p_%s_table[0]);\n",
		p_s_variable_name_prefix, p_s_variable_name_prefix, p_s_variable_name_prefix);
}

inline void CUnicodeMappingDirectory::CMappingDirectoryBuilder::DeleteTable(TTableInfo &r_item)
{
	delete r_item.first;
}

inline void CUnicodeMappingDirectory::CMappingDirectoryBuilder::MakeIdent(char &r_n_char)
{
	if(r_n_char == '-' || (r_n_char != '_' && !isalnum(uint8_t(r_n_char))))
		r_n_char = '_';
}

inline void CUnicodeMappingDirectory::CMappingDirectoryBuilder::ToLower(char &r_n_char)
{
	r_n_char = tolower(uint8_t(r_n_char));
}

/*
 *								=== ~CMappingDirectoryBuilder ===
 */

