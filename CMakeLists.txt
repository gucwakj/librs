# build librs
project (LIBRS)

# set minimum cmake version
cmake_minimum_required (VERSION 2.8)

# set the version number
set (LIBRS_VERSION 0.0.70)
set (LIBRS_SO_VERSION 0)

# verbose ouptut
set (CMAKE_VERBOSE_MAKEFILE off)

# set the build postfix extension
if (CMAKE_BUILD_TYPE MATCHES "Debug")
	set (CMAKE_DEBUG_POSTFIX "d")
endif ()

# set gcc compiler version
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# supress developer warnings
cmake_policy (SET CMP0015 NEW)

# set developer warnings
if (WIN32)
	add_definitions (-D_CRT_SECURE_NO_WARNINGS)
else ()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
endif ()

# include directory
include_directories (
	"${PROJECT_SOURCE_DIR}/include"
	"${PROJECT_BINARY_DIR}"
)

# link directory
link_directories ("${LIBRS_BINARY_DIR}/lib")

# set output directory for shared objects
set (OUTPUT_LIBDIR "${LIBRS_BINARY_DIR}/lib")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
make_directory (${OUTPUT_LIBDIR})

# options for individual robot compilation
set (ENABLE_LINKBOT ON)
set (ENABLE_MINDSTORMS ON)

# add libraries
add_subdirectory ("${LIBRS_SOURCE_DIR}/src")

