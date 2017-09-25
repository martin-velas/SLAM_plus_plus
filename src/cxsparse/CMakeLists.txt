
# Set required variables.
SET(CXSPARSE_LIBRARY_NAME "cxsparse")

FILE(GLOB_RECURSE CXSPARSE_LIBRARY_HEADERS ${SLAM_P_P_SOURCE_DIR}/include/cxsparse/*.h)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include/cxsparse/
    ${PROJECT_SOURCE_DIR}/include/cholmod/SuiteSparse/)

SET(CXSPARSE_LIBRARY_SOURCES
  cs_add.c
  cs_amd.c
  cs_chol.c
  cs_cholsol.c
  cs_compress.c
  cs_counts.c
  cs_cumsum.c
  cs_dfs.c
  cs_dmperm.c
  cs_droptol.c
  cs_dropzeros.c
  cs_dupl.c
  cs_entry.c
  cs_ereach.c
  cs_etree.c
  cs_fkeep.c
  cs_gaxpy.c
  cs_happly.c
  cs_house.c
  cs_ipvec.c
  cs_leaf.c
  cs_load.c
  cs_lsolve.c
  cs_ltsolve.c
  cs_lu.c
  cs_lusol.c
  cs_malloc.c
  cs_maxtrans.c
  cs_multiply.c
  cs_norm.c
  cs_permute.c
  cs_pinv.c
  cs_post.c
  cs_pvec.c
  cs_qr.c
  cs_qrsol.c
  cs_randperm.c
  cs_reach.c
  cs_scatter.c
  cs_scc.c
  cs_schol.c
  cs_spsolve.c
  cs_sqr.c
  cs_symperm.c
  cs_tdfs.c
  cs_transpose.c
  cs_updown.c
  cs_usolve.c
  cs_util.c
  cs_utsolve.c)

# for x86, add NCOMPLEX (for x64, need to add CS_LONG as well)
ADD_DEFINITIONS(-DNCOMPLEX)
IF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD AND NOT SLAM_P_P_SHORT_CXSPARSE)
	ADD_DEFINITIONS(-DCS_LONG)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD AND NOT SLAM_P_P_SHORT_CXSPARSE)

# Build and install SLAM_plus_plus executable.
ADD_LIBRARY(${CXSPARSE_LIBRARY_NAME} STATIC
 ${CXSPARSE_LIBRARY_SOURCES}
 ${CXSPARSE_LIBRARY_HEADERS})
INSTALL(TARGETS ${CXSPARSE_LIBRARY_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
