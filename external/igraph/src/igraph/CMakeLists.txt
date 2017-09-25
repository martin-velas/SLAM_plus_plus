
# Set required variables.
SET(IGRAPH_LIBRARY_NAME "igraph")

FILE(GLOB_RECURSE IGRAPH_LIBRARY_HEADERS ${SLAM_P_P_SOURCE_DIR}/external/igraph/include/igraph/*.h)

INCLUDE_DIRECTORIES(
  ${SLAM_P_P_SOURCE_DIR}/external/igraph/include/igraph/
  ${SLAM_P_P_SOURCE_DIR}/external/igraph/src/igraph/
  ${SLAM_P_P_SOURCE_DIR}/external/igraph/src/igraph/plfit/)

SET(IGRAPH_LIBRARY_SOURCES
  adjlist.c
  attributes.c
  cliques.c
  complex.c
  cores.c
  dqueue.c
  plfit/error.c
  plfit/gss.c
  heap.c
  igraph_error.c
  igraph_set.c
  igraph_stack.c
  interrupt.c
  iterators.c
  plfit/kolmogorov.c
  plfit/lbfgs.c
  math.c
  matrix.c
  maximal_cliques.c
  memory.c
  plfit/options.c
  other.c
  plfit/plfit.c
  progress.c
  qsort.c
  random.c
  structure_generators.c
  type_indexededgelist.c
  types.c
  vector.c
  vector_ptr.c
  version.c
  plfit/zeta.c)

# Build and install igraph library.
ADD_LIBRARY(${IGRAPH_LIBRARY_NAME} STATIC
 ${IGRAPH_LIBRARY_SOURCES}
 ${IGRAPH_LIBRARY_HEADERS})
INSTALL(TARGETS ${IGRAPH_LIBRARY_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
