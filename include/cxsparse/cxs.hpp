/**
 *	@file cxsparse/cxs.hpp
 *	@brief this small file adds namespace around cxsparse to avoid collisions with csparse.
 *	@note The CXSparse library was not modified in any way, with the exception of addition of this file.
 */

namespace cx {

#ifdef CS_LONG
#undef CS_LONG
#endif // CS_LONG
#if defined(_M_X64) || defined(_M_AMD64) || defined(_M_IA64) || defined(__x86_64) || defined(__amd64) || defined(__ia64)
#ifndef __CXSPARSE_SHORT
#define CS_LONG
#endif // !__CXSPARSE_SHORT
#else // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64

#endif // _M_X64 || _M_AMD64 || _M_IA64 || __x86_64 || __amd64 || __ia64
// in x64, we need to use long (otherwise cxsparse uses int, that is too narrow)

#define NCOMPLEX
// cxsparse is compiled with NCOMPLEX flag set

// the above flags are here to configure the cxsparse header. the library is compiled
// with those flags, but to avoid the need to set those flags for all the code that uses
// the library, it's coneniently defined here.

#ifdef CS_VER
#undef CS_VER
#undef CS_SUBVER
#undef CS_SUBSUB
#undef CS_DATE
#undef CS_COPYRIGHT
#endif // CS_VER
// remove colliding defines, noone uses them anyway

#include "cxsparse/cs.h"

#define CXS_VER 2                    /* CXSparse Version */
#define CXS_SUBVER 3
#define CXS_SUBSUB 0
#define CXS_DATE "Jun 1, 2012"       /* CXSparse release date */
#define CXS_COPYRIGHT "Copyright (c) Timothy A. Davis, 2006-2012"
// define CX version, just to retain the copyright and stuffs

#undef cs
#undef cs_add
#undef cs_cholsol
#undef cs_dupl
#undef cs_entry
#undef cs_lusol
#undef cs_gaxpy
#undef cs_multiply
#undef cs_qrsol
#undef cs_transpose
#undef cs_compress
#undef cs_norm
#undef cs_print
#undef cs_load
#undef cs_calloc
#undef cs_free
#undef cs_realloc
#undef cs_spalloc
#undef cs_spfree
#undef cs_sprealloc
#undef cs_malloc
#undef css
#undef csn
#undef csd
#undef cs_amd
#undef cs_chol
#undef cs_dmperm
#undef cs_droptol
#undef cs_dropzeros
#undef cs_happly
#undef cs_ipvec
#undef cs_lsolve
#undef cs_ltsolve
#undef cs_lu
#undef cs_permute
#undef cs_pinv
#undef cs_pvec
#undef cs_qr
#undef cs_schol
#undef cs_sqr
#undef cs_symperm
#undef cs_usolve
#undef cs_utsolve
#undef cs_updown
#undef cs_sfree
#undef cs_nfree
#undef cs_dfree
#undef cs_counts
#undef cs_cumsum
#undef cs_dfs
#undef cs_etree
#undef cs_fkeep
#undef cs_house
#undef cs_invmatch
#undef cs_maxtrans
#undef cs_post
#undef cs_scc
#undef cs_scatter
#undef cs_tdfs
#undef cs_reach
#undef cs_spsolve
#undef cs_ereach
#undef cs_randperm
#undef cs_leaf
#undef cs_dalloc
#undef cs_done
#undef cs_idone
#undef cs_ndone
#undef cs_ddone
// undefine all the symbols defined by CXSparse, because these would collide with CSparse

#ifdef CS_LONG
#ifdef CS_COMPLEX
#define cxs cs_cl
#else // CS_COMPLEX
#define cxs cs_dl
#endif // CS_COMPLEX
#else // CS_LONG
#ifdef CS_COMPLEX
#define cxs cs_ci
#else // CS_COMPLEX
#define cxs cs_di
#endif // CS_COMPLEX
#endif // CS_LONG
#define cxs_add CS_NAME (_add)
#define cxs_cholsol CS_NAME (_cholsol)
#define cxs_dupl CS_NAME (_dupl)
#define cxs_entry CS_NAME (_entry)
#define cxs_lusol CS_NAME (_lusol)
#define cxs_gaxpy CS_NAME (_gaxpy)
#define cxs_multiply CS_NAME (_multiply)
#define cxs_qrsol CS_NAME (_qrsol)
#define cxs_transpose CS_NAME (_transpose)
#define cxs_compress CS_NAME (_compress)
#define cxs_norm CS_NAME (_norm)
#define cxs_print CS_NAME (_print)
#define cxs_load CS_NAME (_load)
#define cxs_calloc CS_NAME (_calloc)
#define cxs_free CS_NAME (_free)
#define cxs_realloc CS_NAME (_realloc)
#define cxs_spalloc CS_NAME (_spalloc)
#define cxs_spfree CS_NAME (_spfree)
#define cxs_sprealloc CS_NAME (_sprealloc)
#define cxs_malloc CS_NAME (_malloc)
#define cxss CS_NAME (s)
#define cxsn CS_NAME (n)
#define cxsd CS_NAME (d)
#define cxs_amd CS_NAME (_amd)
#define cxs_chol CS_NAME (_chol)
#define cxs_dmperm CS_NAME (_dmperm)
#define cxs_droptol CS_NAME (_droptol)
#define cxs_dropzeros CS_NAME (_dropzeros)
#define cxs_happly CS_NAME (_happly)
#define cxs_ipvec CS_NAME (_ipvec)
#define cxs_lsolve CS_NAME (_lsolve)
#define cxs_ltsolve CS_NAME (_ltsolve)
#define cxs_lu CS_NAME (_lu)
#define cxs_permute CS_NAME (_permute)
#define cxs_pinv CS_NAME (_pinv)
#define cxs_pvec CS_NAME (_pvec)
#define cxs_qr CS_NAME (_qr)
#define cxs_schol CS_NAME (_schol)
#define cxs_sqr CS_NAME (_sqr)
#define cxs_symperm CS_NAME (_symperm)
#define cxs_usolve CS_NAME (_usolve)
#define cxs_utsolve CS_NAME (_utsolve)
#define cxs_updown CS_NAME (_updown)
#define cxs_sfree CS_NAME (_sfree)
#define cxs_nfree CS_NAME (_nfree)
#define cxs_dfree CS_NAME (_dfree)
#define cxs_counts CS_NAME (_counts)
#define cxs_cumsum CS_NAME (_cumsum)
#define cxs_dfs CS_NAME (_dfs)
#define cxs_etree CS_NAME (_etree)
#define cxs_fkeep CS_NAME (_fkeep)
#define cxs_house CS_NAME (_house)
#define cxs_invmatch CS_NAME (_invmatch)
#define cxs_maxtrans CS_NAME (_maxtrans)
#define cxs_post CS_NAME (_post)
#define cxs_scc CS_NAME (_scc)
#define cxs_scatter CS_NAME (_scatter)
#define cxs_tdfs CS_NAME (_tdfs)
#define cxs_reach CS_NAME (_reach)
#define cxs_spsolve CS_NAME (_spsolve)
#define cxs_ereach CS_NAME (_ereach)
#define cxs_randperm CS_NAME (_randperm)
#define cxs_leaf CS_NAME (_leaf)
#define cxs_dalloc CS_NAME (_dalloc)
#define cxs_done CS_NAME (_done)
#define cxs_idone CS_NAME (_idone)
#define cxs_ndone CS_NAME (_ndone)
#define cxs_ddone CS_NAME (_ddone)
// define new names with 'x' in them

} // ~cx
