
# recurse
ADD_SUBDIRECTORY(CAMD_DLONG)

# Set required variables.
SET(CHOLMOD_LIBRARY_NAME "cholmod")

FILE(GLOB_RECURSE CHOLMOD_LIBRARY_HEADERS
  ${SLAM_P_P_SOURCE_DIR}/include/cholmod/*.h)

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/include/cholmod/
  ${PROJECT_SOURCE_DIR}/include/cholmod/AMD/
  ${PROJECT_SOURCE_DIR}/include/cholmod/CAMD/
  ${PROJECT_SOURCE_DIR}/include/cholmod/CCOLAMD/
  ${PROJECT_SOURCE_DIR}/include/cholmod/COLAMD/
  ${PROJECT_SOURCE_DIR}/include/cholmod/SuiteSparse/)

# if SUITESPARSE_TIMER_ENABLED is set, it makes some problems
ADD_DEFINITIONS(-DNTIMER)

# support for GPU BLAS
IF(SLAM_P_P_GPU_BLAS)
	ADD_DEFINITIONS(-DGPU_BLAS)
ENDIF(SLAM_P_P_GPU_BLAS)

# support for METIS
IF(SLAM_P_P_USE_METIS)
  INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include/cholmod/METIS/
    ${PROJECT_SOURCE_DIR}/include/cholmod/GKlib/)
  
  # avoid CMAKE warning by the next macro definition (it will blah it is not escaped)
  CMAKE_POLICY(PUSH)
  CMAKE_POLICY(SET CMP0005 OLD)

  # GKlib uses non-standard __thread specifier - ignore it
  ADD_DEFINITIONS(-D__thread=)

  # GKlib has it's own regex header, make it use it (it otherwise includes some other regex which is in turn not found)
  ADD_DEFINITIONS(-DUSE_GKREGEX=)

  # restore any policy that was being set before
  CMAKE_POLICY(POP)
ELSE(SLAM_P_P_USE_METIS)
	ADD_DEFINITIONS(-DNPARTITION)
ENDIF(SLAM_P_P_USE_METIS)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD AND NOT SLAM_P_P_SHORT_CHOLMOD)
	ADD_DEFINITIONS(-DDLONG)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8 AND NOT SLAM_P_P_32BIT_BUILD AND NOT SLAM_P_P_SHORT_CHOLMOD)

SET(CHOLMOD_LIBRARY_SOURCES
  AMD/amd_1.c
  AMD/amd_2.c
  AMD/amd_aat.c
  AMD/amd_control.c
  AMD/amd_defaults.c
  AMD/amd_dump.c
  AMD/amd_global.c
  AMD/amd_info.c
  AMD/amd_order.c
  AMD/amd_postorder.c
  AMD/amd_post_tree.c
  AMD/amd_preprocess.c
  AMD/amd_valid.c
  CAMD/camd_1.c
  CAMD/camd_2.c
  CAMD/camd_aat.c
  CAMD/camd_control.c
  CAMD/camd_defaults.c
  CAMD/camd_dump.c
  CAMD/camd_global.c
  CAMD/camd_info.c
  CAMD/camd_order.c
  CAMD/camd_postorder.c
  CAMD/camd_preprocess.c
  CAMD/camd_valid.c
  CCOLAMD/ccolamd.c
  CCOLAMD/ccolamd_global.c
  CHOLMOD/Check/cholmod_check.c
  CHOLMOD/Check/cholmod_read.c
  CHOLMOD/Check/cholmod_write.c
  CHOLMOD/Cholesky/cholmod_amd.c
  CHOLMOD/Cholesky/cholmod_analyze.c
  CHOLMOD/Cholesky/cholmod_colamd.c
  CHOLMOD/Cholesky/cholmod_etree.c
  CHOLMOD/Cholesky/cholmod_factorize.c
  CHOLMOD/Cholesky/cholmod_postorder.c
  CHOLMOD/Cholesky/cholmod_rcond.c
  CHOLMOD/Cholesky/cholmod_resymbol.c
  CHOLMOD/Cholesky/cholmod_rowcolcounts.c
  CHOLMOD/Cholesky/cholmod_rowfac.c
  CHOLMOD/Cholesky/cholmod_solve.c
  CHOLMOD/Cholesky/cholmod_spsolve.c
  CHOLMOD/Core/cholmod_aat.c
  CHOLMOD/Core/cholmod_add.c
  CHOLMOD/Core/cholmod_band.c
  CHOLMOD/Core/cholmod_change_factor.c
  CHOLMOD/Core/cholmod_common.c
  CHOLMOD/Core/cholmod_complex.c
  CHOLMOD/Core/cholmod_copy.c
  CHOLMOD/Core/cholmod_dense.c
  CHOLMOD/Core/cholmod_error.c
  CHOLMOD/Core/cholmod_factor.c
  CHOLMOD/Core/cholmod_memory.c
  CHOLMOD/Core/cholmod_sparse.c
  CHOLMOD/Core/cholmod_transpose.c
  CHOLMOD/Core/cholmod_triplet.c
  CHOLMOD/Core/cholmod_version.c
  CHOLMOD/MatrixOps/cholmod_drop.c
  CHOLMOD/MatrixOps/cholmod_horzcat.c
  CHOLMOD/MatrixOps/cholmod_norm.c
  CHOLMOD/MatrixOps/cholmod_scale.c
  CHOLMOD/MatrixOps/cholmod_sdmult.c
  CHOLMOD/MatrixOps/cholmod_ssmult.c
  CHOLMOD/MatrixOps/cholmod_submatrix.c
  CHOLMOD/MatrixOps/cholmod_symmetry.c
  CHOLMOD/MatrixOps/cholmod_vertcat.c
  CHOLMOD/Modify/cholmod_rowadd.c
  CHOLMOD/Modify/cholmod_rowdel.c
  CHOLMOD/Modify/cholmod_updown.c
  CHOLMOD/Partition/cholmod_camd.c
  CHOLMOD/Partition/cholmod_ccolamd.c
  CHOLMOD/Partition/cholmod_csymamd.c
  CHOLMOD/Partition/cholmod_metis.c
  CHOLMOD/Partition/cholmod_nesdis.c
  CHOLMOD/Supernodal/cholmod_super_numeric.c
  CHOLMOD/Supernodal/cholmod_super_solve.c
  CHOLMOD/Supernodal/cholmod_super_symbolic.c
  COLAMD/colamd.c
  COLAMD/colamd_global.c
  SuiteSparse/SuiteSparse_config.c)

IF(SLAM_P_P_USE_METIS)
	SET(CHOLMOD_LIBRARY_SOURCES
	  ${CHOLMOD_LIBRARY_SOURCES}
	  METIS/auxapi.c
	  METIS/balance.c
	  METIS/bucketsort.c
	  METIS/checkgraph.c
#	  METIS/ccgraph.c
	  METIS/coarsen.c
	  METIS/compress.c
	  METIS/contig.c
	  METIS/debug.c
#	  METIS/estmem.c
	  METIS/fm.c
	  METIS/fortran.c
	  METIS/frename.c
	  METIS/gklib.c
	  METIS/graph.c
	  METIS/initpart.c
	  METIS/kmetis.c
#	  METIS/kvmetis.c
	  METIS/kwayfm.c
	  METIS/kwayrefine.c
#	  METIS/kwayvolfm.c
#	  METIS/kwayvolrefine.c
	  METIS/mcutil.c
#	  METIS/match.c
#	  METIS/mbalance.c
#	  METIS/mbalance2.c
#	  METIS/mcoarsen.c
#	  METIS/memory.c
	  METIS/mesh.c
	  METIS/meshpart.c
#	  METIS/mfm.c
#	  METIS/mfm2.c
	  METIS/minconn.c
	  METIS/mincover.c
#	  METIS/minitpart.c
#	  METIS/minitpart2.c
#	  METIS/mkmetis.c
#	  METIS/mkwayfmh.c
#	  METIS/mkwayrefine.c
#	  METIS/mmatch.c
	  METIS/mmd.c
#	  METIS/mpmetis.c
#	  METIS/mrefine.c
#	  METIS/mrefine2.c
#	  METIS/mutil.c
#	  METIS/myqsort.c
	  METIS/ometis.c
	  METIS/options.c
	  METIS/parmetis.c
	  METIS/pmetis.c
#	  METIS/pqueue.c
	  METIS/refine.c
	  METIS/separator.c
	  METIS/sfm.c
	  METIS/srefine.c
	  METIS/stat.c
#	  METIS/subdomains.c
	  METIS/timing.c
	  METIS/util.c
	  METIS/wspace.c
	  GKlib/b64.c
	  GKlib/blas.c
	  GKlib/csr.c
	  GKlib/error.c
	  GKlib/evaluate.c
	  GKlib/fkvkselect.c
	  GKlib/fs.c
	  GKlib/getopt.c
	  GKlib/gkregex.c
	  GKlib/graph.c
	  GKlib/htable.c
	  GKlib/io.c
	  GKlib/itemsets.c
	  GKlib/mcore.c
	  GKlib/memory.c
	  GKlib/omp.c
	  GKlib/pdb.c
	  GKlib/pqueue.c
	  GKlib/random.c
	  GKlib/rw.c
	  GKlib/seq.c
	  GKlib/sort.c
	  GKlib/string.c
	  GKlib/timers.c
	  GKlib/tokenizer.c
	  GKlib/util.c)
ENDIF(SLAM_P_P_USE_METIS)

# Build and install SLAM_plus_plus executable.
ADD_LIBRARY(${CHOLMOD_LIBRARY_NAME} STATIC
 ${CHOLMOD_LIBRARY_SOURCES}
 ${CHOLMOD_LIBRARY_HEADERS})
INSTALL(TARGETS ${CHOLMOD_LIBRARY_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
