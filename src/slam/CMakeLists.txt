
# Set required variables.
SET(SLAM_BASE_LIBRARY_NAME "slampp_base")

FILE(GLOB_RECURSE SLAM_BASE_LIBRARY_HEADERS ${SLAM_P_P_SOURCE_DIR}/include/slam/*.h)

IF(SLAM_P_P_SHORT_CXSPARSE)
	ADD_DEFINITIONS(-D__CXSPARSE_SHORT)
ENDIF(SLAM_P_P_SHORT_CXSPARSE)

IF(SLAM_P_P_SHORT_CHOLMOD)
	ADD_DEFINITIONS(-D__CHOLMOD_SHORT)
ENDIF(SLAM_P_P_SHORT_CHOLMOD)

# support for GPU BLAS
IF(SLAM_P_P_GPU_BLAS)
	ADD_DEFINITIONS(
		-DGPU_BLAS
		-D__CHANNEL_DESCRIPTOR_H__
		-D__CUDA_RUNTIME_H__)
ENDIF(SLAM_P_P_GPU_BLAS)

SET(SLAM_BASE_LIBRARY_SOURCES
  LinearSolver_Schur_GPU.cpp
  BlockMatrix.cpp
  BlockMatrixTests.cpp
  Debug.cpp
  Eigenvalues.cpp
  LinearSolver_CholMod.cpp
  LinearSolver_CSparse.cpp
  LinearSolver_CXSparse.cpp
  LinearSolver_Schur.cpp
  OrderingMagic.cpp
  Parser.cpp
  Tags.cpp
  Tga.cpp
  Timer.cpp)

# Build and install SLAM_base library.
ADD_LIBRARY(${SLAM_BASE_LIBRARY_NAME} STATIC
	${SLAM_BASE_LIBRARY_SOURCES}
	${SLAM_BASE_LIBRARY_HEADERS})
INSTALL(TARGETS ${SLAM_BASE_LIBRARY_NAME}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)
