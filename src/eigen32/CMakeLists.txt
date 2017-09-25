
# Set required variables.
SET(EIGEN_LIBRARY_NAME "eigen")

FILE(GLOB_RECURSE EIGEN_LIBRARY_HEADERS ${SLAM_P_P_SOURCE_DIR}/include/eigen/*.h)

INCLUDE_DIRECTORIES(
  ${SLAM_P_P_SOURCE_DIR}/include/eigen32/eigen/
  ${SLAM_P_P_SOURCE_DIR}/include/eigen32/eigen/blas/
  ${SLAM_P_P_SOURCE_DIR}/include/eigen32/eigen/lapack/)

ADD_DEFINITIONS(-DEIGEN_MPL2_ONLY)

IF(MSVC)
  OPTION(BUILD_EIGEN_COMPLEX_BLAS "Build Eigen complex BLAS and LAPACK." FALSE)
  # Visual Studio 2012 has some problem compiling complex files (internal compiler error)
  # this seems to be fixed in other versions
  
  IF(NOT BUILD_EIGEN_COMPLEX_BLAS)
    SET(EIGEN_LIBRARY_SOURCES
      lapack/double.cpp
      lapack/single.cpp
      blas/double.cpp
      blas/single.cpp
      blas/xerbla.cpp)
    MESSAGE(WARNING "Omitting complex Eigen BLAS/LAPACK to avoid Visual Studio 2012 internal compiler error. Enable at your own discretion.")
  ELSE(NOT BUILD_EIGEN_COMPLEX_BLAS)
    SET(EIGEN_LIBRARY_SOURCES
      lapack/complex_double.cpp
      lapack/complex_single.cpp
      lapack/double.cpp
      lapack/single.cpp
      blas/double.cpp
      blas/single.cpp
      blas/complex_double.cpp
      blas/complex_single.cpp
      blas/xerbla.cpp)
  ENDIF(NOT BUILD_EIGEN_COMPLEX_BLAS)
ELSE(MSVC)
  SET(EIGEN_LIBRARY_SOURCES
    lapack/complex_double.cpp
    lapack/complex_single.cpp
    lapack/double.cpp
    lapack/single.cpp
    blas/double.cpp
    blas/single.cpp
    blas/complex_double.cpp
    blas/complex_single.cpp
    blas/xerbla.cpp)
ENDIf(MSVC)

# Build and install SLAM_plus_plus executable.
ADD_LIBRARY(${EIGEN_LIBRARY_NAME} STATIC
 ${EIGEN_LIBRARY_SOURCES}
 ${EIGEN_LIBRARY_HEADERS})
INSTALL(TARGETS ${EIGEN_LIBRARY_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
