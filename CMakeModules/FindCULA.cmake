# this is a minimal find CULA package
# written by Lukas Polok {ipolok@fit.vutbr.cz}
# if CULA_FIND_REQUIRED is set, assumes that CULA is needed and fails if it is not found
# this is written against R16a; the versions change the names of the libs quite a lot
# this sets the following variables:
#
# CULA_ROOT_DIR - root of the CULA installation
# CULA_INCLUDE - path to CULA includes
# CULA_CULA_LIBRARY - filename of CULA core library
# CULA_lapack_LIBRARY - filename of CULA LAPACK library
# CULA_scalapack_LIBRARY - filename of CULA SCALAPACK library
#

# The MIT License
#
# License for the specific language governing rights and limitations under
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
###############################################################################

cmake_policy(PUSH)
cmake_minimum_required(VERSION 2.6.3)
cmake_policy(POP)

IF(NOT CULA_ROOT_DIR)
	# Search in the CULA_ROOT first. note that in linux, it does not have bin folders, only lib
	find_path(CULA_ROOT_DIR
		NAMES libcula_core.so cula_core.dll libcula_lapack.so cula_lapack.dll
		PATHS
			ENV CULA_ROOT
			ENV CULA_BIN_PATH
			ENV CULA_BIN_PATH_32
			ENV CULA_BIN_PATH_64
		PATH_SUFFIXES bin bin64 lib lib64
		DOC "CULA library location."
	)
	IF(CULA_ROOT_DIR)
		IF(UNIX)
			string(REGEX REPLACE "[/\\\\]?lib[64]*[/\\\\]?$" "" CULA_ROOT_DIR ${CULA_ROOT_DIR})
		ELSE(UNIX)
			string(REGEX REPLACE "[/\\\\]?bin[64]*[/\\\\]?$" "" CULA_ROOT_DIR ${CULA_ROOT_DIR})
		ENDIF(UNIX)
		# We need to force this back into the cache.
		set(CULA_ROOT_DIR ${CULA_ROOT_DIR} CACHE PATH "CULA location." FORCE)
	ENDIF(CULA_ROOT_DIR)
	IF(NOT EXISTS ${CULA_ROOT_DIR})
		IF(CULA_FIND_REQUIRED)
			message(FATAL_ERROR "Specify CULA_ROOT_DIR")
		ELSE(CULA_FIND_REQUIRED)
			IF(NOT CULA_FIND_QUIETLY)
				message(STATUS "Specify CULA_ROOT_DIR")
			ENDIF(CULA_FIND_QUIETLY)
		ENDIF(CULA_FIND_REQUIRED)
	ENDIF(NOT EXISTS ${CULA_ROOT_DIR})
ENDIF(NOT CULA_ROOT_DIR)
# find CULA

find_path(CULA_INCLUDE
	cula_lapack.hpp # Header included in toolkit
	PATHS
		"${CULA_ROOT_DIR}"
		ENV CULA_INC_PATH
	PATH_SUFFIXES include
)
find_path(CULA_INCLUDE cula_lapack.hpp)
mark_as_advanced(CULA_INCLUDE)
# find CULA includes

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(CULA_CULA_LIBRARY
		NAMES cula_core
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_64
		PATH_SUFFIXES lib64
		DOC "CULA library location."
	)
	find_library(CULA_lapack_LIBRARY
		NAMES cula_lapack
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_64
		PATH_SUFFIXES lib64
		DOC "CULA library location."
	)
	find_library(CULA_scalapack_LIBRARY
		NAMES cula_scalapack
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_64
		PATH_SUFFIXES lib64
		DOC "CULA library location."
	)
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
	find_library(CULA_CULA_LIBRARY
		NAMES cula_core
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_32
		PATH_SUFFIXES lib
		DOC "CULA library location."
	)
	find_library(CULA_lapack_LIBRARY
		NAMES cula_lapack
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_32
		PATH_SUFFIXES lib
		DOC "CULA library location."
	)
	find_library(CULA_scalapack_LIBRARY
		NAMES cula_scalapack
		PATHS
			ENV CULA_ROOT
			ENV CULA_LIB_PATH_32
		PATH_SUFFIXES lib
		DOC "CULA library location."
	)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)
mark_as_advanced(CULA_CULA_LIBRARY)
mark_as_advanced(CULA_lapack_LIBRARY)
mark_as_advanced(CULA_scalapack_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CULA DEFAULT_MSG
	CULA_ROOT_DIR
	CULA_INCLUDE
	CULA_CULA_LIBRARY
	)

set(CULA_FOUND TRUE)
